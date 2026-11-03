#!/usr/bin/env python3
"""Run real-time BME688 GAS10 inference on the AI PC."""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import threading
import time
import warnings
import zlib
from collections import deque
from pathlib import Path

import joblib
import numpy as np
import torch


PROTOCOL_VERSION = 1
warnings.filterwarnings("ignore", message="torch.meshgrid")
warnings.filterwarnings("ignore", message="Using padding='same'")


def validate_message(message):
    if message.get("version") != PROTOCOL_VERSION:
        raise ValueError(f"unsupported protocol version: {message.get('version')}")
    body = {key: value for key, value in message.items() if key != "crc32"}
    expected = f"{zlib.crc32(json.dumps(body, separators=(',', ':'), sort_keys=True).encode()) & 0xffffffff:08x}"
    if message.get("crc32") != expected:
        raise ValueError("CRC mismatch")
    return message


def ack(seq: int) -> bytes:
    body = {
        "version": PROTOCOL_VERSION,
        "type": "ack",
        "seq": seq,
        "received_at_pc_ns": time.monotonic_ns(),
    }
    return (json.dumps(body, separators=(",", ":")) + "\n").encode()


def load_model(model_path: Path):
    model_dir = model_path.parent
    sys.path.insert(0, str(model_dir))
    from models.model import ConvTran

    meta = json.loads((model_dir / "model_meta.json").read_text(encoding="utf-8"))
    config = {
        "Net_Type": "C-T",
        "Data_shape": [1, 1, int(meta["input_len"])],
        "num_labels": int(meta["num_classes"]),
        "emb_size": int(meta["model"]["emb_size"]),
        "num_heads": int(meta["model"]["num_heads"]),
        "dim_ff": int(meta["model"]["dim_ff"]),
        "dropout": float(meta["model"]["dropout"]),
        "Fix_pos_encode": "tAPE",
        "Rel_pos_encode": "eRPE",
    }
    model = ConvTran(config, num_classes=config["num_labels"])
    with torch.no_grad():
        model(torch.zeros(1, config["Data_shape"][2]))
    model.load_state_dict(torch.load(model_path, map_location="cpu", weights_only=True))
    model.eval()
    return model, joblib.load(model_dir / "scaler.pkl"), meta


def preprocess(values, scaler):
    x = np.asarray(values, dtype=np.float32)
    if x.shape != (10,) or not np.all(np.isfinite(x)) or np.any(x <= 0):
        raise ValueError("GAS10 must contain 10 positive finite values")
    mean = float(x.mean())
    if mean <= 0:
        raise ValueError("GAS10 mean must be positive")
    x = (x / mean).reshape(1, -1)
    return scaler.transform(x).astype(np.float32)


class AirAlcoholPredictor:
    """Reusable one-scan classifier; the network listener remains the CLI's job."""

    def __init__(self, model_path: Path):
        self.model_path = Path(model_path).resolve()
        self.model, self.scaler, self.meta = load_model(self.model_path)
        self.labels = {
            int(item["label"]): item["key"] for item in self.meta["labels"]
        }
        self.model_version = self.model_path.parent.name

    def predict(self, values):
        x = preprocess(values, self.scaler)
        with torch.no_grad():
            logits = self.model(torch.from_numpy(x))[0].detach().cpu().numpy()
        shifted = logits - np.max(logits)
        probs = np.exp(shifted) / np.exp(shifted).sum()
        label = int(np.argmax(logits))
        return {
            "class_key": self.labels[label],
            "probabilities": {
                self.labels[i]: float(probs[i]) for i in range(len(probs))
            },
            "logits": [float(value) for value in logits],
            "model_version": self.model_version,
            "model_path": str(self.model_path),
            "preprocessing": "row_mean_normalize_then_model_scaler",
            "model_profile_equivalence_verified": False,
            "classification_valid": False,
            "status": "inference_unverified_profile",
        }


def percentile(values, q):
    valid = sorted(float(v) for v in values if math.isfinite(float(v)) and float(v) > 0)
    if not valid:
        return None
    if len(valid) == 1:
        return valid[0]
    position = (len(valid) - 1) * q / 100.0
    low = int(position)
    high = min(low + 1, len(valid) - 1)
    fraction = position - low
    return valid[low] * (1.0 - fraction) + valid[high] * fraction


def compute_span(values):
    valid = sorted(float(v) for v in values if math.isfinite(float(v)) and float(v) > 0)
    if len(valid) < 3:
        return 1.0
    p10 = percentile(valid, 10)
    p50 = percentile(valid, 50)
    p90 = percentile(valid, 90)
    if p50 is None or p50 <= 0:
        return 1.0
    return max(0.0, (p90 - p10) / p50)


def is_recovering(r0, last_r0, baseline_r0, last_output, instant_output, air_output):
    if not baseline_r0 or baseline_r0 <= 0 or not last_r0 or last_r0 <= 0:
        return False
    return (
        last_output != air_output
        and instant_output != last_output
        and r0 > last_r0 * 1.03
        and r0 / baseline_r0 < 0.92
    )


def input_listener(stop_event):
    try:
        input("\n预测持续运行中，按 Enter 停止...\n")
    except EOFError:
        pass
    stop_event.set()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--model", required=True)
    parser.add_argument(
        "--samples",
        type=int,
        default=0,
        help="有效样本数；0 表示持续预测，按 Enter 停止",
    )
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--startup-window", type=int, default=20)
    parser.add_argument("--startup-thresh", type=float, default=0.02)
    parser.add_argument("--run-warn-thresh", type=float, default=0.04)
    parser.add_argument("--min-margin", type=float, default=0.80)
    args = parser.parse_args()

    model_path = Path(args.model).resolve()
    model, scaler, meta = load_model(model_path)
    labels = {int(item["label"]): item["key"] for item in meta["labels"]}
    counts = {label: 0 for label in labels}
    r0_window = deque(maxlen=max(1, args.startup_window))
    baseline_r0 = None
    started = False
    warn_unstable = False
    recovering = False
    last_r0 = None
    air_output = labels.get(0, "air")
    last_output = air_output
    output_counts = {}
    output_logits = []
    stop_event = threading.Event()
    listener = threading.Thread(target=input_listener, args=(stop_event,), daemon=True)
    listener.start()

    print(f"模型：{model_path}")
    print(f"连接 PYNQ：{args.host}:{args.port}")
    print(
        f"连续预测：先等待空气基线稳定 "
        f"(R0 波动<{args.startup_thresh * 100:.0f}%，窗口={args.startup_window})，"
        f"运行波动阈值={args.run_warn_thresh * 100:.0f}%，"
        f"logit margin阈值={args.min_margin:.2f}"
    )
    with socket.create_connection((args.host, args.port), timeout=args.timeout) as sock:
        sock.settimeout(0.5)
        receive_buffer = b""
        while not stop_event.is_set() and (
            args.samples <= 0 or sum(counts.values()) < args.samples
        ):
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                raise ConnectionError("PYNQ closed the connection")
            receive_buffer += chunk
            while b"\n" in receive_buffer:
                raw, _, receive_buffer = receive_buffer.partition(b"\n")
                message = validate_message(json.loads(raw))
                if message["type"] == "hello":
                    print("PYNQ 已连接，等待 GAS10 样本...")
                    continue
                if message["type"] != "sample":
                    continue

                seq = int(message["seq"])
                sock.sendall(ack(seq))
                payload = message.get("payload", {})
                values = payload.get("gas10_ohm")
                print(
                    f"\n[{seq}] timestamp_ns={message.get('timestamp_ns')} "
                    f"valid={payload.get('valid')} "
                    f"mask={payload.get('gas_valid_mask')} "
                    f"T={payload.get('temperature_c')}C "
                    f"RH={payload.get('humidity_rh')}% "
                    f"P={payload.get('pressure_hpa')}hPa"
                )
                if isinstance(values, list):
                    print("    GAS10[ohm]: " + " ".join(
                        f"{float(v):.1f}" if v is not None else "NA" for v in values
                    ))
                if (
                    payload.get("feature_type") != "GAS10"
                    or not payload.get("valid")
                    or payload.get("gas_valid_mask") != 1023
                ):
                    print("    状态：无效样本，跳过推理")
                    continue

                x = preprocess(values, scaler)
                raw_values = np.asarray(values, dtype=np.float32)
                r0 = float(raw_values[0])
                r0_window.append(r0)
                span = compute_span(list(r0_window))
                if not started:
                    if len(r0_window) < args.startup_window:
                        print(
                            f"    [启动中] R0={r0:.1f} "
                            f"积累 {len(r0_window)}/{args.startup_window}"
                        )
                        last_r0 = r0
                        continue
                    if span < args.startup_thresh:
                        started = True
                        baseline_r0 = percentile(list(r0_window), 50)
                        print(f"    [OK] air_baseline≈{baseline_r0:.1f} ohm，进入连续预测")
                    else:
                        print(f"    [启动中] R0波动={span * 100:.2f}%，等待稳定空气基线")
                        last_r0 = r0
                        continue
                warn_unstable = span >= args.run_warn_thresh
                if baseline_r0 and r0 / baseline_r0 >= 0.92 and not warn_unstable:
                    recovering = False
                previous_r0 = last_r0
                last_r0 = r0
                with torch.no_grad():
                    logits = model(torch.from_numpy(x))[0].detach().cpu().numpy()
                label = int(np.argmax(logits))
                counts[label] += 1
                ordered_logits = sorted(logits, reverse=True)
                margin = ordered_logits[0] - ordered_logits[1] if len(ordered_logits) > 1 else 0.0
                instant_output = labels.get(label, str(label))
                if is_recovering(
                    r0,
                    previous_r0,
                    baseline_r0,
                    last_output,
                    instant_output,
                    air_output,
                ):
                    recovering = True
                if recovering:
                    status = "恢复中"
                    output = "恢复中"
                elif warn_unstable:
                    status = "波动较大"
                    output = instant_output
                    last_output = instant_output
                elif margin < args.min_margin:
                    status = "低置信度"
                    output = instant_output
                    last_output = instant_output
                else:
                    status = "实时"
                    output = instant_output
                    last_output = instant_output
                output_counts[output] = output_counts.get(output, 0) + 1
                output_logits.append(float(logits[label]))
                print(
                    f"    输出:{output}  瞬时:{instant_output} [{status}] "
                    f"R0={r0:.1f} air_ratio={r0 / baseline_r0:.3f} "
                    f"span={span * 100:.2f}%\n"
                    f"    logits={' '.join(f'{v:.3f}' for v in logits)} "
                    f"margin={margin:.3f} (阈值 {args.min_margin:.2f})；"
                    f"瞬时累计={counts} 稳定逻辑输出累计={output_counts}"
                )
                if args.samples > 0 and sum(counts.values()) >= args.samples:
                    stop_event.set()
                    break

    if not counts or sum(counts.values()) == 0:
        print("\n未获得有效 GAS10 样本，未生成预测结果。")
        return 1
    print("\n预测结果")
    if output_counts:
        final_output = max(output_counts, key=output_counts.get)
        print(f"参考逻辑输出众数：{final_output}，计数={output_counts}")
    else:
        print("尚无符合参考逻辑的稳定分类输出。")
    print(f"累计样本：{ {labels[k]: v for k, v in counts.items()} }")
    if output_logits:
        print(f"平均最大 logit（不是概率）：{np.mean(output_logits):.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
