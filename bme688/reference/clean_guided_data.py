"""
Clean guided_collect.py CSV files and export a trainable dataset.

This script is intentionally conservative about low resistance values. Some
real odors can sit in the thousands, so low absolute R values are not removed
unless the user explicitly asks for a low floor.

Examples:
    python clean_guided_data.py --in data_0701.csv
    python clean_guided_data.py --in 0701dataset --train-out dataset_0701_train.csv
"""

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd


R_COLS = [f"R{i}" for i in range(10)]
META_COLS = [
    "timestamp",
    "session_id",
    "round_idx",
    "class_key",
    "class_name",
    "ambient_temp_c",
    "ambient_humidity_rh",
    "odor_distance_cm",
    "baseline_r0",
    "qc_ok",
    "qc_flags",
    "source_file",
]
HARD_QC_FLAGS = {
    "bad_length",
    "zero_or_negative",
    "constant_like",
    "abnormal_scale",
    "abnormal_high_scale",
    "possible_saturation_or_fault",
}
PREFERRED_CLASS_ORDER = [
    "air",
    "alcohol",
    "toilet",
    "vinegar",
    "bleach",
    "exhaust",
    "oil",
]
DEFAULT_CLASS_NAMES = {
    "air": "AIR",
    "alcohol": "ALCOHOL",
    "toilet": "TOILET",
    "vinegar": "VINEGAR",
    "bleach": "BLEACH",
    "exhaust": "EXHAUST",
    "oil": "OIL",
}


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="input_path", required=True, help="guided CSV file or folder of CSV files")
    ap.add_argument("--clean-out", help="clean CSV with metadata")
    ap.add_argument("--train-out", help="train CSV")
    ap.add_argument("--summary-out", help="JSON cleaning summary")
    ap.add_argument("--classes-out", help="classes.json path")
    ap.add_argument("--classes", help="comma-separated class_key allowlist/order, e.g. air,alcohol,vinegar")
    ap.add_argument("--no-balance", action="store_true", help="do not downsample to equal class counts")
    ap.add_argument("--drop-inter-seq-jump", action="store_true", help="drop qc_flags containing inter_sequence_jump")
    ap.add_argument("--low-floor", type=float, default=0.0, help="optional strict low floor; 0 disables it")
    ap.add_argument("--high-ceil", type=float, default=1e8, help="drop rows with any R above this value")
    ap.add_argument("--outlier-z", type=float, default=10.0, help="log row-mean MAD z threshold; <=0 disables it")
    ap.add_argument("--seed", type=int, default=42)
    return ap.parse_args()


def default_paths(input_path):
    src = Path(input_path)
    if src.is_dir():
        base = src.parent / src.name
    else:
        base = src.with_suffix("")
    return (
        base.with_name(f"{base.name}_clean.csv"),
        base.with_name(f"{base.name}_train.csv"),
        base.with_name(f"{base.name}_summary.json"),
        base.with_name(f"{base.name}_classes.json"),
    )


def csv_files(input_path):
    src = Path(input_path)
    if src.is_dir():
        files = sorted(src.glob("*.csv"))
        if not files:
            raise ValueError(f"No CSV files found in {src}")
        return files
    if not src.exists():
        raise FileNotFoundError(src)
    return [src]


def split_flags(value):
    if pd.isna(value) or str(value).strip() == "":
        return set()
    return {part.strip() for part in str(value).split("|") if part.strip()}


def load_inputs(input_path):
    frames = []
    summaries = []
    for path in csv_files(input_path):
        df = pd.read_csv(path)
        if df.empty:
            raise ValueError(f"{path} has no data rows")
        rename_cols = {}
        for i in range(10):
            upper = f"R{i}"
            lower = f"r{i}"
            if upper not in df.columns and lower in df.columns:
                rename_cols[lower] = upper
        if rename_cols:
            df = df.rename(columns=rename_cols)
        missing_r = [col for col in R_COLS if col not in df.columns]
        if missing_r:
            raise ValueError(f"{path} missing resistance columns: {missing_r}")
        if "class_key" not in df.columns:
            if "label" not in df.columns:
                raise ValueError(f"{path} missing required column: class_key")
            df["class_key"] = df["label"].apply(lambda value: f"class_{int(value)}")
        df = df.copy()
        df["source_file"] = path.name
        if "qc_flags" not in df.columns:
            df["qc_flags"] = ""
        if "qc_ok" not in df.columns:
            df["qc_ok"] = 1
        if "class_name" not in df.columns:
            df["class_name"] = df["class_key"]
        frames.append(df)
        summaries.append({"file": path.name, "raw_rows": int(len(df))})
    return pd.concat(frames, ignore_index=True), summaries


def ordered_classes(df, explicit_classes=None):
    available = list(dict.fromkeys(df["class_key"].astype(str).tolist()))
    if explicit_classes:
        requested = [item.strip() for item in explicit_classes.split(",") if item.strip()]
        present = [key for key in requested if key in available]
        if present:
            return list(dict.fromkeys(present))
        raise ValueError(f"Requested classes not present in data. requested={requested}, available={available}")

    preferred = [key for key in PREFERRED_CLASS_ORDER if key in available]
    remaining = [key for key in available if key not in preferred]
    return preferred + remaining


def class_display_name(df, class_key):
    if class_key in DEFAULT_CLASS_NAMES:
        return DEFAULT_CLASS_NAMES[class_key]
    rows = df[df["class_key"] == class_key]
    if not rows.empty and "class_name" in rows.columns:
        value = str(rows.iloc[0].get("class_name", "")).strip()
        if value and value.lower() != "nan":
            return value
    return DEFAULT_CLASS_NAMES.get(class_key, class_key)


def hard_filter(df, low_floor, high_ceil, drop_inter_seq_jump):
    work = df.copy()
    work[R_COLS] = work[R_COLS].apply(pd.to_numeric, errors="coerce")
    arr = work[R_COLS].to_numpy(dtype=float)

    finite_ok = np.isfinite(arr).all(axis=1)
    positive_ok = (arr > 0).all(axis=1)
    high_ok = (arr < high_ceil).all(axis=1)
    low_ok = np.ones(len(work), dtype=bool)
    if low_floor and low_floor > 0:
        low_ok = (arr > low_floor).all(axis=1)

    flags = work["qc_flags"].fillna("").astype(str)
    hard_ok = ~flags.apply(lambda s: bool(split_flags(s) & HARD_QC_FLAGS)).to_numpy()
    if drop_inter_seq_jump:
        hard_ok &= ~flags.apply(lambda s: "inter_sequence_jump" in split_flags(s)).to_numpy()

    mask = finite_ok & positive_ok & high_ok & low_ok & hard_ok
    return work.loc[mask].copy(), mask


def remove_relative_outliers(df, z_thresh):
    if z_thresh <= 0 or df.empty:
        return df.copy(), 0

    kept = []
    removed = 0
    for _, group in df.groupby(["source_file", "class_key"], dropna=False):
        if len(group) < 30:
            kept.append(group)
            continue

        row_mean = group[R_COLS].mean(axis=1).to_numpy(dtype=float)
        log_mean = np.log(row_mean)
        med = np.median(log_mean)
        mad = np.median(np.abs(log_mean - med))
        if mad <= 0:
            kept.append(group)
            continue

        robust_z = np.abs(log_mean - med) / (1.4826 * mad)
        mask = robust_z <= z_thresh
        removed += int((~mask).sum())
        kept.append(group.loc[mask].copy())

    if not kept:
        return df.iloc[0:0].copy(), removed
    return pd.concat(kept, ignore_index=True), removed


def remap_labels(df, classes):
    label_map = {key: i for i, key in enumerate(classes)}
    out = df[df["class_key"].isin(classes)].copy()
    out["label"] = out["class_key"].map(label_map).astype(int)
    return out, label_map


def balance_classes(df, seed):
    if df.empty or df["label"].nunique() <= 1:
        return df.copy(), {}
    counts = df["label"].value_counts().sort_index()
    min_count = int(counts.min())
    parts = [
        group.sample(n=min_count, random_state=seed)
        for _, group in df.groupby("label", sort=True)
    ]
    balanced = pd.concat(parts, ignore_index=True).sample(frac=1.0, random_state=seed)
    return balanced.reset_index(drop=True), {str(k): int(v) for k, v in counts.items()}


def main():
    args = parse_args()
    default_clean, default_train, default_summary, default_classes = default_paths(args.input_path)
    clean_out = Path(args.clean_out) if args.clean_out else default_clean
    train_out = Path(args.train_out) if args.train_out else default_train
    summary_out = Path(args.summary_out) if args.summary_out else default_summary
    classes_out = Path(args.classes_out) if args.classes_out else default_classes

    raw, file_summaries = load_inputs(args.input_path)
    classes = ordered_classes(raw, args.classes)
    filtered_by_class = raw[raw["class_key"].isin(classes)].copy()
    hard_clean, hard_mask = hard_filter(
        filtered_by_class,
        low_floor=args.low_floor,
        high_ceil=args.high_ceil,
        drop_inter_seq_jump=args.drop_inter_seq_jump,
    )
    outlier_clean, outlier_removed = remove_relative_outliers(hard_clean, args.outlier_z)
    remapped, label_map = remap_labels(outlier_clean, classes)
    final_df, pre_balance_counts = (
        (remapped.copy(), {})
        if args.no_balance
        else balance_classes(remapped, args.seed)
    )

    if final_df.empty:
        raise ValueError("No usable samples after cleaning")

    meta_cols = [col for col in META_COLS if col in final_df.columns]
    output_cols = meta_cols + R_COLS + ["label"]
    final_df = final_df[output_cols].copy()
    final_df.to_csv(clean_out, index=False, encoding="utf-8-sig")
    final_df.to_csv(train_out, index=False, encoding="utf-8-sig")

    classes_payload = {
        "classes": [
            {
                "label": int(label_map[key]),
                "key": key,
                "name": class_display_name(raw, key),
                "enabled": True,
                "class_type": "odor",
            }
            for key in classes
        ]
    }
    classes_out.write_text(json.dumps(classes_payload, ensure_ascii=False, indent=2), encoding="utf-8")

    summary = {
        "input": str(args.input_path),
        "raw_rows": int(len(raw)),
        "class_filter_rows": int(len(filtered_by_class)),
        "after_hard_filter_rows": int(len(hard_clean)),
        "relative_outlier_removed": int(outlier_removed),
        "after_outlier_rows": int(len(remapped)),
        "final_rows": int(len(final_df)),
        "balanced": not args.no_balance,
        "pre_balance_counts": pre_balance_counts,
        "final_counts_by_class": final_df["class_key"].value_counts().to_dict(),
        "label_map": classes_payload["classes"],
        "files": file_summaries,
        "hard_flags": sorted(HARD_QC_FLAGS),
        "low_floor": args.low_floor,
        "high_ceil": args.high_ceil,
        "outlier_z": args.outlier_z,
    }
    summary_out.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"Raw rows: {len(raw)}")
    print(f"After hard filter: {len(hard_clean)}")
    print(f"After outlier filter: {len(remapped)}")
    print(f"Final rows: {len(final_df)}")
    print("Final class counts:")
    print(final_df["class_key"].value_counts().sort_index().to_string())
    print(f"Clean CSV: {clean_out}")
    print(f"Train CSV: {train_out}")
    print(f"Summary JSON: {summary_out}")
    print(f"Classes JSON: {classes_out}")


if __name__ == "__main__":
    main()
