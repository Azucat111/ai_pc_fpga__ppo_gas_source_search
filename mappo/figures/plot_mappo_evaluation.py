#!/usr/bin/env python3
"""Render Chinese publication-style MAPPO evaluation figures from frozen JSON."""

import csv
import json
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OUT = Path(__file__).resolve().parent


def wilson_interval(successes, trials, z=1.959963984540054):
    p = successes / trials
    denom = 1 + z * z / trials
    center = (p + z * z / (2 * trials)) / denom
    half = z * np.sqrt(p * (1 - p) / trials + z * z / (4 * trials * trials)) / denom
    return center - half, center + half


def load_json(name):
    return json.loads((ROOT / "artifacts" / name).read_text(encoding="utf-8"))


def make_rows():
    rows = []
    definitions = [
        ("主评估", "30 m 场景；10 个智能体、10 个静态源；目标为找到至少 3 个",
         ["current_normal3_20.json"]),
        ("主评估", "30 m 场景；动态源比例 0.3；找到全部 10 个；MAPPO + 覆盖先验",
         ["current_normal10_prior20.json"]),
        ("主评估", "50 m 迁移场景；找到全部 20 个；MAPPO + 覆盖先验",
         ["current_transfer20_confirm1_threshold075_20.json",
          "current_transfer20_confirm1_threshold075_10b.json"]),
        ("半径敏感性", "50 m 配对评估；成功半径 1.5 m",
         ["paired_radius150_confirm1_threshold075_20.json"]),
        ("半径敏感性", "50 m 配对评估；成功半径 2.0 m",
         ["paired_radius200_confirm1_threshold075_20.json"]),
    ]
    for group, label, filenames in definitions:
        records = [load_json(name) for name in filenames]
        episodes = [episode for record in records for episode in record["episodes"]]
        n = len(episodes)
        k = sum(bool(episode["success"]) for episode in episodes)
        low, high = wilson_interval(k, n)
        rows.append({
            "group": group,
            "label": label,
            "successes": k,
            "episodes": n,
            "success_rate": k / n,
            "wilson95_low": low,
            "wilson95_high": high,
            "source_files": ";".join(filenames),
        })
    return rows


def write_data(rows):
    with (OUT / "mappo_success_rates.csv").open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    manifest = {
        "figure": "mappo_evaluation",
        "scope": "仿真评估；各配置独立呈现，不代表实车或跨场景泛化结论",
        "interval": "95% Wilson score interval for a binomial proportion",
        "aggregation": "只汇总配置一致的迁移评估两份记录；其余配置分别计算",
        "sources": sorted({name for row in rows for name in row["source_files"].split(";")}),
        "rows": rows,
    }
    (OUT / "mappo_evaluation_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def draw(rows):
    mpl.rcParams.update({
        "font.family": "Noto Sans CJK SC", "font.size": 8.5,
        "axes.labelsize": 8.5, "axes.titlesize": 9.5,
        "xtick.labelsize": 8, "ytick.labelsize": 8,
        "pdf.fonttype": 42, "ps.fonttype": 42, "svg.fonttype": "none",
        "axes.spines.top": False, "axes.spines.right": False,
    })
    fig, ax = plt.subplots(figsize=(8.4, 4.55), layout="constrained")
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    y_positions = [4.25, 3.35, 2.45, 1.15, 0.25]
    colors = {"主评估": "#0072B2", "半径敏感性": "#D55E00"}
    for row, y in zip(rows, y_positions):
        rate = row["success_rate"] * 100
        low, high = row["wilson95_low"] * 100, row["wilson95_high"] * 100
        ax.errorbar(rate, y, xerr=[[rate - low], [high - rate]], fmt="o",
                    color=colors[row["group"]], ecolor=colors[row["group"]],
                    markersize=5, capsize=3.5, linewidth=1.4, zorder=3)
        ax.text(102, y, f"{row['successes']}/{row['episodes']}  |  {low:.1f}%–{high:.1f}%",
                ha="left", va="center", fontsize=7.8, color="#263238")

    ax.axhline(1.8, color="#B9C2C9", linewidth=0.8, linestyle=(0, (3, 3)))
    ax.text(-0.02, 4.78, "A  主评估", transform=ax.get_yaxis_transform(),
            ha="left", va="bottom", fontsize=9, fontweight="bold", color="#263238")
    ax.text(-0.02, 1.55, "B  配对半径敏感性", transform=ax.get_yaxis_transform(),
            ha="left", va="bottom", fontsize=9, fontweight="bold", color="#263238")
    ax.set_yticks(y_positions, [row["label"] for row in rows])
    ax.tick_params(axis="y", length=0, pad=8)
    ax.set_xlim(0, 150)
    ax.set_ylim(-0.35, 5.02)
    ax.set_xticks([0, 25, 50, 75, 100])
    ax.set_xlabel("成功回合比例（%）；误差线为 Wilson 95% 置信区间")
    ax.grid(axis="x", color="#D9DEE3", linewidth=0.6, zorder=0)
    ax.spines["left"].set_visible(False)
    ax.spines["bottom"].set_color("#73808A")
    ax.text(102, 4.86, "成功数/回合数  |  95% 区间", ha="left", va="bottom",
            fontsize=7.2, color="#59666F")
    fig.suptitle("MAPPO 多智能体气味源定位仿真评估", fontsize=11, fontweight="bold")
    for fmt in ("pdf", "svg", "png"):
        kwargs = {"format": fmt, "facecolor": "white"}
        if fmt == "png":
            kwargs["dpi"] = 600
        fig.savefig(OUT / f"mappo_evaluation.{fmt}", **kwargs)
    plt.close(fig)


def main():
    rows = make_rows()
    write_data(rows)
    draw(rows)
    print(json.dumps({"output_dir": str(OUT), "rows": rows}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
