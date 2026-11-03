#!/usr/bin/env python3
"""Filter the reference cleaned dataset to air/alcohol and remap labels."""

import argparse
import json
from pathlib import Path

import pandas as pd


PROJECT_ROOT = Path(__file__).resolve().parents[2]
REFERENCE_ROOT = PROJECT_ROOT / "bme688" / "reference"
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True,
                        help="External cleaned dataset containing class_key and label")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    frame = pd.read_csv(args.input)
    if not {"class_key", "label"}.issubset(frame.columns):
        raise ValueError("Input must contain class_key and label columns")
    frame = frame[frame["class_key"].isin(["air", "alcohol"])].copy()
    if set(frame["class_key"].unique()) != {"air", "alcohol"}:
        raise ValueError("Input must include both air and alcohol classes")
    frame["label"] = frame["class_key"].map({"air": 0, "alcohol": 1}).astype(int)
    counts = frame["class_key"].value_counts().to_dict()
    if counts.get("air") != counts.get("alcohol"):
        raise ValueError(f"Expected balanced binary data, got {counts}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    frame.to_csv(args.output, index=False, encoding="utf-8-sig")
    classes_path = args.output.parent / "classes.json"
    classes_path.write_text(
        json.dumps(
            {
                "classes": [
                    {
                        "label": 0,
                        "key": "air",
                        "name": "AIR",
                        "enabled": True,
                        "class_type": "odor",
                    },
                    {
                        "label": 1,
                        "key": "alcohol",
                        "name": "ALCOHOL",
                        "enabled": True,
                        "class_type": "odor",
                    },
                ]
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    print(f"Input: {args.input}")
    print(f"Rows: {len(frame)}; class counts: {counts}")
    print(f"Binary dataset: {args.output}")
    print(f"Classes: {classes_path}")


if __name__ == "__main__":
    main()
