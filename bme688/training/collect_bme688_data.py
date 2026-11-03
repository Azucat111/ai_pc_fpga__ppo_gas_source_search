#!/usr/bin/env python3
"""Train and evaluate a GAS10 ConvTran model from Dashboard capture sessions.

The current single-session data is split chronologically within each class.
This gives a leakage-resistant within-session check, not an independent-session
generalization estimate. Preprocessing and ConvTran architecture follow the
Odor Workbench project.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import random
import shutil
import sys
from datetime import datetime
from pathlib import Path

import joblib
import numpy as np
import pandas as pd
import torch
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import (
    balanced_accuracy_score,
    classification_report,
    confusion_matrix,
    f1_score,
)
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC
from torch.utils.data import DataLoader, TensorDataset


ROOT = Path(__file__).resolve().parents[2]
REFERENCE = ROOT / "bme688" / "reference"
FEATURES = [f"R{i}" for i in range(10)]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--data-dir", type=Path,
        default=ROOT / "bme688/data",
    )
    parser.add_argument(
        "--output-dir", type=Path,
        default=ROOT / "bme688/model/new_run",
    )
    parser.add_argument("--epochs", type=int, default=300)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--patience", type=int, default=40)
    return parser.parse_args()


def set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.use_deterministic_algorithms(True, warn_only=True)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_and_clean(data_dir: Path) -> tuple[pd.DataFrame, dict, list[str]]:
    sys.path.insert(0, str(REFERENCE))
    import clean_guided_data as cleaner

    paths = sorted(data_dir.glob("session_*.csv"))
    if not paths:
        raise ValueError(f"No session_*.csv capture files found under {data_dir}")

    frames = []
    input_hashes = {}
    for path in paths:
        frame = pd.read_csv(path, encoding="utf-8-sig")
        missing = [name for name in FEATURES + ["class_key", "label"] if name not in frame]
        if missing:
            raise ValueError(f"{path.name} missing columns: {missing}")
        frame["source_file"] = path.name
        frames.append(frame)
        input_hashes[path.name] = sha256(path)

    raw = pd.concat(frames, ignore_index=True)
    loaded_rows = int(len(raw))
    supplied_labels = pd.to_numeric(raw["label"], errors="coerce")
    raw["label"] = supplied_labels
    if "qc_ok" in raw.columns:
        qc_mask = pd.to_numeric(raw["qc_ok"], errors="coerce").fillna(0) == 1
        qc_rejected_rows = int((~qc_mask).sum())
        raw = raw[qc_mask].copy()
    else:
        qc_rejected_rows = 0

    class_labels = raw.groupby("class_key")["label"].nunique(dropna=True)
    if len(class_labels) < 2 or (class_labels != 1).any():
        raise ValueError("Need at least two classes, each with exactly one consistent integer label")
    label_by_key = raw.groupby("class_key")["label"].first().astype(int).to_dict()
    if sorted(label_by_key.values()) != list(range(len(label_by_key))):
        raise ValueError("Class labels must be unique and contiguous from 0")
    class_keys = [key for key, _ in sorted(label_by_key.items(), key=lambda item: item[1])]

    hard_clean, hard_mask = cleaner.hard_filter(
        raw, low_floor=0.0, high_ceil=1e8, drop_inter_seq_jump=False
    )
    clean, outliers_removed = cleaner.remove_relative_outliers(
        hard_clean, z_thresh=10.0
    )
    if clean.empty:
        raise ValueError("No valid samples remain after reference cleaning")
    clean["label"] = clean["class_key"].map(label_by_key).astype(int)
    clean["timestamp"] = pd.to_datetime(clean["timestamp"], errors="coerce")
    clean = clean.dropna(subset=["timestamp"]).copy()
    clean = clean.sort_values(["class_key", "timestamp"], kind="stable").reset_index(drop=True)

    summary = {
        "input_files": [path.name for path in paths],
        "input_sha256": input_hashes,
        "raw_rows": loaded_rows,
        "qc_rejected_rows": qc_rejected_rows,
        "after_capture_qc_rows": int(len(raw)),
        "after_hard_filter_rows": int(len(hard_clean)),
        "hard_filter_removed_rows": int(len(raw) - len(hard_clean)),
        "relative_outlier_removed_rows": int(outliers_removed),
        "clean_rows": int(len(clean)),
        "class_counts": {key: int((clean["class_key"] == key).sum()) for key in class_keys},
        "cleaning_source": str(REFERENCE / "clean_guided_data.py"),
        "cleaning_rules": {
            "positive_finite_gas_resistance": True,
            "qc_hard_flags_removed": sorted(cleaner.HARD_QC_FLAGS),
            "low_floor": 0.0,
            "high_ceil": 1e8,
            "relative_outlier_mad_z": 10.0,
        },
    }
    return clean, summary, class_keys


def chronological_split(frame: pd.DataFrame, class_keys: list[str]) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict, str]:
    sessions_by_class = [
        set(frame.loc[frame["class_key"] == key, "session_id"].astype(str))
        for key in class_keys
    ]
    all_sessions = set(frame["session_id"].astype(str))
    common_sessions = set.intersection(*sessions_by_class) if sessions_by_class else set()
    if len(common_sessions) >= 3 and common_sessions == all_sessions:
        session_times = frame.assign(session_sort=frame["timestamp"]).groupby(
            frame["session_id"].astype(str)
        )["session_sort"].max().sort_values()
        sessions = session_times.index.tolist()
        n_train = max(1, int(len(sessions) * 0.6))
        n_validation = max(1, int(len(sessions) * 0.2))
        if n_train + n_validation >= len(sessions):
            n_train = len(sessions) - 2
            n_validation = 1
        train_sessions = set(sessions[:n_train])
        validation_sessions = set(sessions[n_train:n_train + n_validation])
        test_sessions = set(sessions[n_train + n_validation:])
        train_idx = frame.index[frame["session_id"].astype(str).isin(train_sessions)].to_numpy()
        validation_idx = frame.index[frame["session_id"].astype(str).isin(validation_sessions)].to_numpy()
        test_idx = frame.index[frame["session_id"].astype(str).isin(test_sessions)].to_numpy()
        counts = {
            key: {
                part: int(((frame.loc[idx, "class_key"] == key).sum()))
                for part, idx in (("train", train_idx), ("validation", validation_idx), ("test", test_idx))
            }
            for key in class_keys
        }
        if all(counts[key][part] > 0 for key in class_keys for part in ("train", "validation", "test")):
            return train_idx, validation_idx, test_idx, {
                "strategy": "chronological_session_holdout",
                "sessions": {"train": sorted(train_sessions), "validation": sorted(validation_sessions), "test": sorted(test_sessions)},
                "counts_by_class": counts,
            }, "chronological_session_holdout"

    parts = {"train": [], "validation": [], "test": []}
    counts = {}
    for key in class_keys:
        indices = frame.index[frame["class_key"] == key].to_numpy()
        n = len(indices)
        if n < 30:
            raise ValueError(f"Need at least 30 cleaned rows per class; {key} has {n}")
        n_train = int(n * 0.60)
        n_validation = int(n * 0.20)
        train_end = n_train
        validation_end = n_train + n_validation
        parts["train"].extend(indices[:train_end])
        parts["validation"].extend(indices[train_end:validation_end])
        parts["test"].extend(indices[validation_end:])
        counts[key] = {
            "total": n,
            "train": train_end,
            "validation": validation_end - train_end,
            "test": n - validation_end,
            "train_time": [str(frame.loc[indices[0], "timestamp"]), str(frame.loc[indices[train_end - 1], "timestamp"])],
            "validation_time": [str(frame.loc[indices[train_end], "timestamp"]), str(frame.loc[indices[validation_end - 1], "timestamp"])],
            "test_time": [str(frame.loc[indices[validation_end], "timestamp"]), str(frame.loc[indices[-1], "timestamp"])],
        }
    arrays = tuple(np.asarray(parts[name], dtype=int) for name in ("train", "validation", "test"))
    for indices in arrays:
        indices.sort()
    return arrays[0], arrays[1], arrays[2], {
        "strategy": "chronological_within_class_60_20_20",
        "counts_by_class": counts,
    }, "chronological_within_class_60_20_20"


def normalize_rows(values: np.ndarray) -> np.ndarray:
    means = values.mean(axis=1, keepdims=True)
    if np.any(means <= 0) or not np.isfinite(means).all():
        raise ValueError("GAS10 row means must be positive and finite")
    return values / means


def build_model(sample_count: int, num_classes: int):
    sys.path.insert(0, str(REFERENCE))
    from models.model import ConvTran

    config = {
        "Net_Type": "C-T",
        "Data_shape": [sample_count, 1, 10],
        "num_labels": num_classes,
        "emb_size": 32,
        "num_heads": 2,
        "dim_ff": 128,
        "dropout": 0.1,
        "Fix_pos_encode": "tAPE",
        "Rel_pos_encode": "eRPE",
    }
    model = ConvTran(config, num_classes=num_classes)
    with torch.no_grad():
        model(torch.zeros(1, 10))
    return model, config


def evaluate(model, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    model.eval()
    with torch.no_grad():
        logits = model(torch.from_numpy(x.astype(np.float32)))
        probabilities = torch.softmax(logits, dim=1).numpy()
    return logits.argmax(dim=1).numpy(), probabilities


def report_metrics(y_true: np.ndarray, predictions: np.ndarray,
                   probabilities: np.ndarray, labels: list[int],
                   class_names: list[str], alcohol_label: int | None) -> dict:
    result = {
        "accuracy": float((y_true == predictions).mean()),
        "balanced_accuracy": float(balanced_accuracy_score(y_true, predictions)),
        "macro_f1": float(f1_score(y_true, predictions, average="macro", zero_division=0)),
        "confusion_matrix_labels": confusion_matrix(y_true, predictions, labels=labels).tolist(),
        "class_order": class_names,
        "classification_report": classification_report(
            y_true, predictions, labels=labels, target_names=class_names,
            output_dict=True, zero_division=0,
        ),
    }
    if alcohol_label is not None:
        alcohol_col = labels.index(alcohol_label)
        positive_scores = probabilities[:, alcohol_col]
        air_mask = y_true == labels[class_names.index("air")] if "air" in class_names else np.zeros_like(y_true, dtype=bool)
        result["workflow_threshold_diagnostic"] = {
            "alcohol_probability_threshold": 0.8,
            "alcohol_test_samples_above_threshold": int(((y_true == alcohol_label) & (positive_scores > 0.8)).sum()),
            "alcohol_test_sample_count": int((y_true == alcohol_label).sum()),
            "air_test_samples_above_threshold": int((air_mask & (positive_scores > 0.8)).sum()),
            "air_test_sample_count": int(air_mask.sum()),
        }
    return result


def main() -> int:
    args = parse_args()
    if args.epochs < 1 or args.patience < 1:
        raise ValueError("epochs and patience must be positive")
    set_seed(args.seed)
    torch.set_num_threads(2)
    args.output_dir = args.output_dir.resolve()
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        raise FileExistsError(f"Refusing to overwrite existing training artifacts: {args.output_dir}")
    args.output_dir.mkdir(parents=True, exist_ok=True)

    clean, summary, class_keys = load_and_clean(args.data_dir.resolve())
    train_idx, validation_idx, test_idx, split_summary, split_strategy = chronological_split(clean, class_keys)
    label_by_key = clean.groupby("class_key")["label"].first().astype(int).to_dict()
    labels = list(range(len(class_keys)))
    class_names = ["alcohol" if key == "alc" else key for key in class_keys]
    alcohol_key = next((key for key in class_keys if key.lower() in {"alcohol", "alc"}), None)
    alcohol_label = label_by_key.get(alcohol_key) if alcohol_key is not None else None
    if len(set(class_names)) != len(class_names):
        raise ValueError("Class labels collide after mapping alc to alcohol")
    raw = clean[FEATURES].to_numpy(dtype=np.float32)
    x = normalize_rows(raw)
    y = clean["label"].to_numpy(dtype=np.int64)
    scaler = StandardScaler().fit(x[train_idx])
    x_scaled = scaler.transform(x).astype(np.float32)
    joblib.dump(scaler, args.output_dir / "scaler.pkl")
    clean.to_csv(args.output_dir / "cleaned_capture_data.csv", index=False, encoding="utf-8-sig")

    x_train, y_train = x_scaled[train_idx], y[train_idx]
    x_validation, y_validation = x_scaled[validation_idx], y[validation_idx]
    x_test, y_test = x_scaled[test_idx], y[test_idx]

    model, config = build_model(len(y_train), len(class_keys))
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
    criterion = torch.nn.CrossEntropyLoss()
    loader = DataLoader(
        TensorDataset(torch.from_numpy(x_train), torch.from_numpy(y_train)),
        batch_size=32, shuffle=True, generator=torch.Generator().manual_seed(args.seed),
    )
    best_score = (-1.0, -1.0, float("-inf"))
    best_epoch = 0
    stale_epochs = 0
    best_path = args.output_dir / "best_model.pth"
    log_rows = []

    for epoch in range(1, args.epochs + 1):
        model.train()
        losses = []
        for xb, yb in loader:
            optimizer.zero_grad()
            loss = criterion(model(xb), yb)
            loss.backward()
            optimizer.step()
            losses.append(float(loss.item()))
        val_pred, _ = evaluate(model, x_validation, y_validation)
        val_f1 = float(f1_score(y_validation, val_pred, average="macro", zero_division=0))
        val_accuracy = float((val_pred == y_validation).mean())
        model.eval()
        with torch.no_grad():
            val_loss = float(criterion(
                model(torch.from_numpy(x_validation)),
                torch.from_numpy(y_validation),
            ).item())
        score = (val_f1, val_accuracy, -val_loss)
        log_rows.append({
            "epoch": epoch,
            "train_loss": float(np.mean(losses)),
            "validation_accuracy": val_accuracy,
            "validation_macro_f1": val_f1,
            "validation_loss": val_loss,
        })
        if score > best_score:
            best_score = score
            best_epoch = epoch
            stale_epochs = 0
            torch.save(model.state_dict(), best_path)
        else:
            stale_epochs += 1
        if epoch == 1 or epoch % 20 == 0:
            print(f"epoch={epoch} loss={np.mean(losses):.4f} val_acc={val_accuracy:.3f} val_macro_f1={val_f1:.3f}")
        if stale_epochs >= args.patience:
            break

    model.load_state_dict(torch.load(best_path, map_location="cpu", weights_only=True))
    y_pred, probabilities = evaluate(model, x_test, y_test)
    convtran_metrics = report_metrics(y_test, y_pred, probabilities, labels, class_names, alcohol_label)

    baselines = {}
    for name, estimator in (
        ("logistic_regression", LogisticRegression(max_iter=2000, class_weight="balanced", random_state=args.seed)),
        ("rbf_svc", SVC(C=1.0, kernel="rbf", class_weight="balanced",
                         probability=True, random_state=args.seed)),
    ):
        baseline = type(estimator)(**estimator.get_params())
        baseline.fit(x_train, y_train)
        baseline_pred = baseline.predict(x_test)
        baseline_proba = baseline.predict_proba(x_test)
        baselines[name] = report_metrics(y_test, baseline_pred, baseline_proba, labels, class_names, alcohol_label)

    label_metadata = [
        {"label": label_by_key[key], "key": "alcohol" if key == "alc" else key,
         "name": ("alcohol" if key == "alc" else str(clean.loc[clean.class_key == key, "class_name"].iloc[0])),
         "class_type": "odor"}
        for key in class_keys
    ]
    meta = {
        "schema_version": 1,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "generated_by": "collect_bme688_data.py",
        "source_data_dir": str(args.data_dir.resolve()),
        "source_files_sha256": summary["input_sha256"],
        "feature_columns": FEATURES,
        "input_len": 10,
        "num_classes": 2,
        "labels": label_metadata,
        "label_keys": [item["key"] for item in label_metadata],
        "label_names": [item["name"] for item in label_metadata],
        "normalize_baseline": True,
        "preprocessing": "row_mean_normalize_then_standard_scaler_fit_on_train_only",
        "model": {"architecture": "ConvTran", "emb_size": 32, "num_heads": 2,
                  "dim_ff": 128, "dropout": 0.1},
        "split": {
            **split_summary,
            "session_ids": sorted(clean["session_id"].astype(str).unique().tolist()),
            "independent_session_test": split_strategy == "chronological_session_holdout",
        },
        "metrics": convtran_metrics,
        "best_epoch": best_epoch,
    }
    (args.output_dir / "model_meta.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
    metrics = {
        "seed": args.seed,
        "best_epoch": best_epoch,
        "epochs_run": len(log_rows),
        "cleaning": summary,
        "split": meta["split"],
        "convtran_test": convtran_metrics,
        "baselines_test": baselines,
        "caveat": (
            "A chronological independent-session holdout was used, but more dates, environments, and physical repetitions are still needed."
            if meta["split"]["independent_session_test"] else
            "No independent-session holdout was possible. This chronological block test must not be treated as deployment generalization evidence."
        ),
    }
    (args.output_dir / "metrics.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8")
    with (args.output_dir / "training_log.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=log_rows[0].keys())
        writer.writeheader()
        writer.writerows(log_rows)

    predictions = clean.loc[test_idx, ["timestamp", "session_id", "class_key", "label"]].copy()
    names_by_label = dict(zip(labels, class_names))
    predictions["true_class"] = [names_by_label[int(value)] for value in y_test]
    predictions["predicted_class"] = [names_by_label[int(value)] for value in y_pred]
    for label, class_name in zip(labels, class_names):
        predictions[f"probability_{class_name}"] = probabilities[:, label]
    predictions.to_csv(args.output_dir / "test_predictions.csv", index=False, encoding="utf-8-sig")
    with (args.output_dir / "classification_report.txt").open("w", encoding="utf-8") as stream:
        stream.write("ConvTran chronological-block test\n")
        stream.write(classification_report(y_test, y_pred, labels=labels, target_names=class_names, zero_division=0))
        stream.write(f"\nConfusion matrix (true rows, predicted columns; {', '.join(class_names)}):\n")
        stream.write(np.array2string(np.asarray(convtran_metrics["confusion_matrix_labels"])))
        stream.write("\n\nCaveat: all samples are from one session; this is not an independent-session test.\n")

    threshold = convtran_metrics.get("workflow_threshold_diagnostic")
    _, all_probabilities = evaluate(model, x_scaled, y)
    alcohol_column = labels.index(alcohol_label) if alcohol_label is not None else None
    alcohol_rows = y == alcohol_label if alcohol_label is not None else np.zeros_like(y, dtype=bool)
    alcohol_above_threshold = int((all_probabilities[alcohol_rows, alcohol_column] > 0.8).sum()) if alcohol_column is not None else 0
    alcohol_below_threshold = int(alcohol_rows.sum()) - alcohol_above_threshold
    class_counts_text = ", ".join(f"{name}: {summary['class_counts'][key]}" for key, name in zip(class_keys, class_names))
    split_counts_text = ", ".join(f"{name} {convtran_metrics['classification_report'][name]['support']:.0f}" for name in class_names)
    threshold_text = (
        f"酒精 {threshold['alcohol_test_samples_above_threshold']}/{threshold['alcohol_test_sample_count']} 超阈值；"
        f"空气误超阈值 {threshold['air_test_samples_above_threshold']}/{threshold['air_test_sample_count']}"
        if threshold else "当前标签中没有 air/alcohol 对，未计算工作流 alcohol>0.8 阈值诊断"
    )
    scope_text = (
        "本次测试按完整 session 留出，仍需更多日期、环境和重复采集验证。"
        if meta["split"]["independent_session_test"] else
        f"当前只有 {len(meta['split']['session_ids'])} 个 session，测试采用时间块留出，不是独立 session、跨日期或跨环境验证。"
    )
    alcohol_capture_text = (
        f"`{alcohol_key}` 全量回看中 {alcohol_below_threshold} 条未达到 0.8，其余 {alcohol_above_threshold} 条超过 0.8；"
        "该回看包含训练数据，仅用于发现采集边界问题，不是独立评估。请核对暴露开始后的传感器过渡记录。"
        if alcohol_key is not None else
        "当前标签中没有 alcohol/alc 类别，因此未进行工作流酒精阈值回看。"
    )
    report_text = f"""# Dashboard GAS10 初训与测试报告

## 结论

ConvTran 已训练并通过当前数据划分测试：{len(test_idx)} 条测试样本中正确 {int((y_pred == y_test).sum())} 条，Accuracy、Balanced Accuracy、Macro-F1 均为 {convtran_metrics['accuracy']:.3f}。{scope_text}当前模型只能视为实验候选，不能据此宣称已完成生产验收。

## 数据与处理

- 输入：`{', '.join(summary['input_files'])}`；类别样本数：{class_counts_text}；session 为 `{', '.join(meta['split']['session_ids'])}`。
- 清洗：复用 `bme688/reference/clean_guided_data.py` 的硬质量标记、正值/有限值/量程和行均值 MAD 异常值规则；本次共 {summary['raw_rows']} 行，采集 QC 剔除 {summary['qc_rejected_rows']} 行，硬错误再剔除 {summary['hard_filter_removed_rows']} 行，相对异常值剔除 {summary['relative_outlier_removed_rows']} 行。
- 特征：GAS10 的 R0-R9，每行除以自身均值，再使用仅在训练集拟合的 `StandardScaler`。
- 划分：{split_strategy}。训练 {len(train_idx)} 条、验证 {len(validation_idx)} 条、测试 {len(test_idx)} 条；类别测试支持数：{split_counts_text}。
- 模型：沿用参考工程 ConvTran（10 点输入，32 维 embedding、2 heads、128 FFN、dropout 0.1）；seed={args.seed}，最佳 epoch={best_epoch}。

## 测试结果

混淆矩阵（真实类别为行，预测类别为列；{', '.join(class_names)}）：

```text
{np.array2string(np.asarray(convtran_metrics['confusion_matrix_labels']))}
```

- 工作流概率阈值诊断（`alcohol > 0.8`）：{threshold_text}。
- Logistic Regression 与 RBF-SVM 基线结果见 `metrics.json`。小样本条件下，基线对比不能单独证明 ConvTran 更优。

## 风险与后续

- {alcohol_capture_text}
- 独立 session 测试状态：`{meta['split']['independent_session_test']}`。若为 false，后续应按多次独立采集、重新建立空气基线的 session 留出测试。
- `predict_air_alcohol.py` 可加载本模型，但当前推理器将自训练模型标记为 `classification_valid=false` / `inference_unverified_profile`。这次没有修改该安全/有效性门控；模型不能直接作为工作流已验收分类器使用。
- 标签 `alc` 在模型元数据中映射为工作流使用的 `alcohol`。其他自定义标签保留用户标签；状态机目前仍只按 `alcohol` 概率确认酒精，不能把其他类别模型直接视作已接入相应状态机。

## 复现

```bash
python3 bme688/training/collect_bme688_data.py \\
  --data-dir bme688/data \\
  --output-dir bme688/model/new_run \\
  --epochs 300 --patience 40 --seed 42
```

指标、源 CSV SHA-256、清洗摘要和分区时间范围见 `metrics.json`；逐样本测试输出见 `test_predictions.csv`。模型权重、scaler 与 `models/` 推理结构保存在本目录。

## 方法工具参考

Kassis, T., Agarwal, V., He, Y., Patel, D., & Brueckner, A. M. (2026). *Scientific Agent Skills: A Library of Procedural Knowledge for Research Agents* (v2). arXiv:2609.00065. https://doi.org/10.48550/arXiv.2609.00065
"""
    (args.output_dir / "training_report.md").write_text(report_text, encoding="utf-8")

    models_dir = args.output_dir / "models"
    models_dir.mkdir(exist_ok=True)
    for source in (REFERENCE / "models").glob("*.py"):
        shutil.copy2(source, models_dir / source.name)

    print(json.dumps({
        "output_dir": str(args.output_dir.resolve()),
        "train_samples": int(len(train_idx)),
        "validation_samples": int(len(validation_idx)),
        "test_samples": int(len(test_idx)),
        "best_epoch": best_epoch,
        "convtran_test": {
            key: convtran_metrics[key]
            for key in ("accuracy", "balanced_accuracy", "macro_f1", "confusion_matrix_labels")
        },
        "baselines_test": {
            key: {metric: value[metric] for metric in ("accuracy", "balanced_accuracy", "macro_f1")}
            for key, value in baselines.items()
        },
        "independent_session_test": meta["split"]["independent_session_test"],
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
