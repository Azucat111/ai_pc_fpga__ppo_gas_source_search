#!/usr/bin/env python3
"""Aggregate independent evaluation JSON files without selecting a best seed.

All inputs are produced by evaluate_mappo.py.  The script is intentionally
read-only with respect to the inputs and writes no artifact unless --output is
provided, so it can be used as a submission evidence audit.
"""
from __future__ import annotations

import argparse
import glob
import json
import math
from pathlib import Path


def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    if n <= 0:
        return 0.0, 0.0
    p = k / n
    d = 1.0 + z * z / n
    c = (p + z * z / (2.0 * n)) / d
    h = z * math.sqrt(p * (1.0 - p) / n + z * z / (4.0 * n * n)) / d
    return c, h


def load(paths: list[str]) -> dict:
    rows = []
    for pattern in paths:
        matches = sorted(glob.glob(pattern)) or [pattern]
        for name in matches:
            path = Path(name)
            payload = json.loads(path.read_text(encoding="utf-8"))
            summary = payload.get("summary", payload)
            episodes = int(summary.get("episodes", len(payload.get("episodes", []))))
            rate = float(summary.get("success_rate", 0.0))
            successes = int(round(rate * episodes))
            center, half = wilson(successes, episodes)
            rows.append({
                "file": str(path), "episodes": episodes,
                "successes": successes, "success_rate": rate,
                "success_ci95": [max(0.0, center - half), min(1.0, center + half)],
                "mean_found": summary.get("mean_found"),
                "mean_time_s": summary.get("mean_time_s"),
                "mean_path_length": summary.get("mean_path_length"),
                "communication": summary.get("communication"),
                "coverage_prior": summary.get("coverage_prior"),
            })
    if not rows:
        raise SystemExit("no evaluation JSON files matched")
    n = sum(r["episodes"] for r in rows)
    k = sum(r["successes"] for r in rows)
    center, half = wilson(k, n)
    return {"files": rows, "pooled": {
        "episodes": n, "successes": k, "success_rate": k / n,
        "success_ci95": [max(0.0, center - half), min(1.0, center + half)],
    }}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("files", nargs="+", help="JSON paths or glob patterns")
    parser.add_argument("--output", help="optional JSON output path inside MARL/new")
    args = parser.parse_args()
    result = load(args.files)
    rendered = json.dumps(result, indent=2, ensure_ascii=False)
    print(rendered)
    if args.output:
        out = Path(args.output)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(rendered + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
