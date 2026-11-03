#!/usr/bin/env python3
"""Train the BME688 air/alcohol classifier using session-level holdout."""

from __future__ import annotations

import argparse
import csv
import json
import os
import random
import shutil
import sys
from datetime import datetime
from pathlib import Path

import joblib
import numpy as np
import pandas as pd
import torch
from sklearn.metrics import classification_report, confusion_matrix, f1_score
from sklearn.preprocessing import StandardScaler
from torch import nn
from torch.utils.data import DataLoader, TensorDataset


PROJECT_ROOT = Path(__file__).resolve().parents[2]
REFERENCE_ROOT = PROJECT_ROOT / "bme688" / "reference"
DATA_ROOT = PROJECT_ROOT / "bme688" / "data"
MODEL_SOURCE = REFERENCE_ROOT / "models"
DEFAULT_TRAIN_FILES = [
    "session_20261031_212430_air.csv",
    "session_20261101_194845_alc.csv",
    "session_20261101_200345_alc.csv",
]
DEFAULT_EVAL_FILES = [
    "session_20261101_185808_air.csv",
    "session_20261101_200745_alc.csv",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        default=str(PROJECT_ROOT / "bme688" / "model" / "new_session_run"),
    )
    parser.add_argument("--data-dir", type=Path, default=DATA_ROOT)
    parser.add_argument("--train-files", nargs="+", default=DEFAULT_TRAIN_FILES)
    parser.add_argument("--eval-files", nargs="+", default=DEFAULT_EVAL_FILES)
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--threads", type=int, default=4)
    return parser.parse_args()


def set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)


def read_files(names: list[str], data_dir: Path) -> pd.DataFrame:
    frames = []
    for name in names:
        path = data_dir / name
        if not path.exists():
            raise FileNotFoundError(path)
        frame = pd.read_csv(path)
        frame["source_file"] = name
        frames.append(frame)
    return pd.concat(frames, ignore_index=True)


def prepare_features(frame: pd.DataFrame) -> np.ndarray:
    feature_cols = [f"R{i}" for i in range(10)]
    missing = [col for col in feature_cols if col not in frame.columns]
    if missing:
        raise ValueError(f"Missing feature columns: {missing}")
    x = frame[feature_cols].to_numpy(dtype=np.float32)
    row_mean = x.mean(axis=1, keepdims=True)
    row_mean[row_mean == 0] = 1.0
    return x / row_mean


def make_loader(x: np.ndarray, y: np.ndarray, batch_size: int, shuffle: bool) -> DataLoader:
    dataset = TensorDataset(torch.from_numpy(x.copy()), torch.from_numpy(y.copy()))
    return DataLoader(dataset, batch_size=batch_size, shuffle=shuffle)


def evaluate(model: nn.Module, loader: DataLoader, device: torch.device):
    model.eval()
    predictions = []
    targets = []
    with torch.no_grad():
        for x_batch, y_batch in loader:
            logits = model(x_batch.to(device))
            predictions.append(logits.argmax(1).cpu())
            targets.append(y_batch)
    pred = torch.cat(predictions).numpy()
    target = torch.cat(targets).numpy()
    return pred, target


def main() -> int:
    args = parse_args()
    set_seed(args.seed)
    torch.set_num_threads(max(1, args.threads))
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    # One complete acquisition session per class is held out for evaluation.
    train_files = args.train_files
    eval_files = args.eval_files
    if set(train_files) & set(eval_files):
        raise ValueError("Training and evaluation files must be disjoint")
    train_df = read_files(train_files, args.data_dir)
    eval_df = read_files(eval_files, args.data_dir)
    for frame in (train_df, eval_df):
        if "qc_ok" in frame:
            frame.drop(frame.index[pd.to_numeric(frame["qc_ok"], errors="coerce").fillna(0) != 1], inplace=True)
    if set(train_df["label"].unique()) != {0, 1} or set(eval_df["label"].unique()) != {0, 1}:
        raise ValueError("Both train and held-out files must contain air=0 and alcohol=1")

    x_train_raw = prepare_features(train_df)
    x_eval_raw = prepare_features(eval_df)
    y_train = train_df["label"].to_numpy(dtype=np.int64)
    y_eval = eval_df["label"].to_numpy(dtype=np.int64)
    if sorted(np.unique(y_train).tolist()) != [0, 1]:
        raise ValueError("Expected binary labels air=0 and alcohol=1 in training data")

    scaler = StandardScaler()
    x_train = scaler.fit_transform(x_train_raw).astype(np.float32)
    x_eval = scaler.transform(x_eval_raw).astype(np.float32)
    joblib.dump(scaler, output_dir / "scaler.pkl")

    device = torch.device(args.device if args.device != "auto" else "cpu")
    if device.type == "cuda" and not torch.cuda.is_available():
        device = torch.device("cpu")

    # Import the reference architecture without modifying the original project.
    sys.path.insert(0, str(REFERENCE_ROOT))
    from models.model import ConvTran

    config = {
        "Net_Type": "C-T",
        "Data_shape": [len(y_train), 1, 10],
        "num_labels": 2,
        "emb_size": 32,
        "num_heads": 2,
        "dim_ff": 128,
        "dropout": 0.1,
        "Fix_pos_encode": "tAPE",
        "Rel_pos_encode": "eRPE",
    }
    model = ConvTran(config, num_classes=2).to(device)
    # ConvTran creates its first convolution lazily on the first forward pass.
    with torch.no_grad():
        model(torch.from_numpy(x_train[:1]).to(device))

    train_loader = make_loader(x_train, y_train, args.batch_size, shuffle=True)
    eval_loader = make_loader(x_eval, y_eval, args.batch_size, shuffle=False)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    criterion = nn.CrossEntropyLoss()

    best_acc = -1.0
    best_f1 = -1.0
    best_epoch = 0
    best_path = output_dir / "best_model.pth"
    log_rows = []

    for epoch in range(1, args.epochs + 1):
        model.train()
        losses = []
        for x_batch, y_batch in train_loader:
            optimizer.zero_grad()
            loss = criterion(model(x_batch.to(device)), y_batch.to(device))
            loss.backward()
            optimizer.step()
            losses.append(float(loss.item()))

        pred, target = evaluate(model, eval_loader, device)
        acc = float((pred == target).mean())
        macro_f1 = float(f1_score(target, pred, average="macro", zero_division=0))
        log_rows.append(
            {
                "epoch": epoch,
                "train_loss": f"{np.mean(losses):.6f}",
                "held_out_session_accuracy": f"{acc:.6f}",
                "held_out_session_macro_f1": f"{macro_f1:.6f}",
            }
        )
        if (macro_f1, acc) > (best_f1, best_acc):
            best_acc, best_f1, best_epoch = acc, macro_f1, epoch
            torch.save(model.state_dict(), best_path)
        if epoch == 1 or epoch % 25 == 0 or epoch == args.epochs:
            print(
                f"epoch {epoch:3d}/{args.epochs} "
                f"loss={np.mean(losses):.4f} acc={acc:.4f} f1={macro_f1:.4f}"
            )

    with best_path.open("rb") as stream:
        model.load_state_dict(torch.load(stream, map_location=device, weights_only=True))
    pred, target = evaluate(model, eval_loader, device)
    cm = confusion_matrix(target, pred, labels=[0, 1])
    report = classification_report(
        target,
        pred,
        labels=[0, 1],
        target_names=["air", "alcohol"],
        digits=4,
        zero_division=0,
    )

    with (output_dir / "training_log.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=log_rows[0].keys())
        writer.writeheader()
        writer.writerows(log_rows)
    pd.DataFrame(cm, index=["air", "alcohol"], columns=["air", "alcohol"]).to_csv(
        output_dir / "confusion_matrix.csv"
    )
    (output_dir / "classification_report.txt").write_text(report, encoding="utf-8")

    labels = [
        {"label": 0, "key": "air", "name": "air", "class_type": "odor"},
        {"label": 1, "key": "alcohol", "name": "alcohol", "class_type": "odor"},
    ]
    meta = {
        "schema_version": 1,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "source_dataset_dir": "bme688/data",
        "train_files": train_files,
        "held_out_evaluation_files": eval_files,
        "num_classes": 2,
        "input_len": 10,
        "feature_columns": [f"R{i}" for i in range(10)],
        "normalize_baseline": True,
        "labels": labels,
        "label_keys": ["air", "alcohol"],
        "label_names": ["air", "alcohol"],
        "model": {
            "architecture": "ConvTran",
            "emb_size": 32,
            "num_heads": 2,
            "dim_ff": 128,
            "dropout": 0.1,
        },
        "metrics": {
            "best_epoch": best_epoch,
            "held_out_session_accuracy": float((pred == target).mean()),
            "held_out_session_macro_f1": float(
                f1_score(target, pred, average="macro", zero_division=0)
            ),
            "samples": {"train": len(y_train), "evaluation": len(y_eval)},
            "confusion_matrix": cm.tolist(),
        },
    }
    (output_dir / "model_meta.json").write_text(
        json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    (output_dir / "metrics.json").write_text(
        json.dumps(meta["metrics"], ensure_ascii=False, indent=2), encoding="utf-8"
    )

    # Keep the exact architecture beside the deployable model for PC inference.
    deploy_models = output_dir / "models"
    deploy_models.mkdir(exist_ok=True)
    for source in MODEL_SOURCE.glob("*.py"):
        shutil.copy2(source, deploy_models / source.name)

    print("\nFinal held-out-session evaluation")
    print(report)
    print(f"model: {best_path}")
    print(f"metadata: {output_dir / 'model_meta.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
