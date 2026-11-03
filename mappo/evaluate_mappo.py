#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[2]
PACKAGE_DIR = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))
from env import LearnedMARLEnv, make_training_config
from mappo_core import CentralizedCritic, HeteroMAPPOActor


def evaluate(args):
    ckpt = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    actor = HeteroMAPPOActor(); actor.load_state_dict(ckpt["actor"]); actor.eval()
    cfg = make_training_config(args.scenario, agents=args.agents, max_seconds=args.max_seconds,
                               required_sources=args.required_sources,
                               dynamic_source_fraction=args.dynamic_source_fraction,
                               success_radius=args.success_radius)
    rows = []
    for i in range(args.episodes):
        episode_seed = args.seed + i
        env = LearnedMARLEnv(cfg, episode_seed, communication=not args.no_communication,
                             packet_loss=args.packet_loss,
                             message_aggregation=args.message_aggregation,
                             coverage_prior=args.coverage_prior,
                             coverage_pattern=args.coverage_pattern,
                             start_layout=args.start_layout,
                             confirmation_threshold_scale=args.confirmation_threshold_scale,
                             confirmation_steps=args.confirmation_steps)
        hist, roles, recv, aux = env.reset()
        done = False
        last = None
        while not done:
            with torch.no_grad():
                a, m, *_ = actor.sample(torch.tensor(hist, dtype=torch.float32),
                                        torch.tensor(roles), torch.tensor(recv, dtype=torch.float32),
                                        torch.tensor(aux, dtype=torch.float32), deterministic=True)
            (hist, roles, recv, aux), _, done, last = env.step(a.numpy(), m.numpy())
        rows.append({"episode_seed": episode_seed, **last})
    summary = {"episodes": len(rows), "success_rate": float(np.mean([x["success"] for x in rows])),
               "mean_found": float(np.mean([x["found"] for x in rows])),
               "mean_time_s": float(np.mean([x["time_s"] for x in rows])),
               "mean_path_length": float(np.mean([x["path_length"] for x in rows])),
               "mean_safety_overrides": float(np.mean([x["safety_overrides"] for x in rows])),
               "mean_coverage_overrides": float(np.mean([x.get("coverage_overrides", 0) for x in rows])),
               "total_coverage_overrides": int(np.sum([x.get("coverage_overrides", 0) for x in rows])),
               "communication": not args.no_communication, "packet_loss": args.packet_loss}
    summary["coverage_prior"] = args.coverage_prior
    payload = {
        "algorithm": "hetero_mappo_v1",
        "scenario": args.scenario,
        "config": vars(cfg),
        "evaluation": {
            "checkpoint": str(Path(args.checkpoint)),
            "episodes": args.episodes,
            "episode_seed_start": args.seed,
            "episode_seed_end": args.seed + args.episodes - 1,
            "deterministic_policy": True,
            "communication": not args.no_communication,
            "packet_loss": args.packet_loss,
            "message_aggregation": args.message_aggregation,
            "coverage_prior": args.coverage_prior,
            "coverage_pattern": args.coverage_pattern,
            "start_layout": args.start_layout,
            "confirmation_threshold_scale": args.confirmation_threshold_scale,
            "confirmation_steps": args.confirmation_steps,
        },
        "summary": summary,
        "episodes": rows,
    }
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output).write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", required=True)
    p.add_argument("--scenario", default="competition_20m_10agents")
    p.add_argument("--agents", type=int, default=5)
    p.add_argument("--max-seconds", type=float, default=60.0)
    p.add_argument("--required-sources", type=int, default=None)
    p.add_argument("--dynamic-source-fraction", type=float, default=None)
    p.add_argument("--success-radius", type=float, default=None)
    p.add_argument("--episodes", type=int, default=20)
    p.add_argument("--packet-loss", type=float, default=0.0)
    p.add_argument("--message-aggregation", choices=("mean", "max_response"), default="mean")
    p.add_argument("--coverage-prior", action="store_true")
    p.add_argument("--coverage-pattern", choices=("radial", "radial_dense", "radial_slow", "radial_hard", "radial_wide", "radial_wide_hard", "radial_transfer", "lawnmower", "grid_dense", "raster_dense", "role_split"), default="radial")
    p.add_argument("--no-communication", action="store_true")
    p.add_argument("--start-layout", choices=("center", "ring"), default="center")
    p.add_argument("--confirmation-threshold-scale", type=float, default=1.0)
    p.add_argument("--confirmation-steps", type=int, default=2)
    p.add_argument("--seed", type=int, default=20262000)
    p.add_argument("--output", default="MARL/new/artifacts/eval.json")
    evaluate(p.parse_args())
