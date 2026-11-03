#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch
from torch import optim

ROOT = Path(__file__).resolve().parents[2]
PACKAGE_DIR = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))
from env import LearnedMARLEnv, make_training_config
from mappo_core import CentralizedCritic, HeteroMAPPOActor, MESSAGE_DIM


def onehot(roles, n=4):
    return torch.nn.functional.one_hot(roles, n).float()


def train(args):
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    device = torch.device(args.device)
    cfg = make_training_config(args.scenario, agents=args.agents, max_seconds=args.max_seconds,
                               required_sources=args.required_sources,
                               dynamic_source_fraction=args.dynamic_source_fraction,
                               success_radius=args.success_radius)
    env = LearnedMARLEnv(cfg, args.seed, communication=not args.no_communication,
                         packet_loss=args.packet_loss,
                         message_aggregation=args.message_aggregation,
                         coverage_prior=args.coverage_prior,
                         coverage_pattern=args.coverage_pattern,
                         start_layout=args.start_layout,
                         confirmation_threshold_scale=args.confirmation_threshold_scale,
                         confirmation_steps=args.confirmation_steps,
                         coverage_progress_coef=args.coverage_progress_coef,
                         response_delta_coef=args.response_delta_coef,
                         diversity_coef=args.diversity_coef,
                         duplicate_coef=args.duplicate_coef)
    actor = HeteroMAPPOActor().to(device)
    critic = CentralizedCritic().to(device)
    loaded = 0
    if args.resume:
        previous = torch.load(args.resume, map_location=device, weights_only=False)
        actor.load_state_dict(previous["actor"])
        if previous.get("critic") is not None:
            critic.load_state_dict(previous["critic"])
            loaded = -2  # actor and centralized critic restored
        else:
            loaded = -1  # legacy actor-only checkpoint
    if args.base_ppo:
        loaded = actor.load_single_agent_checkpoint(args.base_ppo)
        actor.to(device)
    opt = optim.Adam(list(actor.parameters()) + list(critic.parameters()), lr=args.lr)
    logs = []
    for episode in range(args.episodes):
        # Domain randomization is applied at episode boundaries so the actor
        # sees both the normal map and the larger transfer map while retaining
        # the same normalized 21D observation contract.  This avoids making a
        # separate target-domain policy and tests whether the representation
        # actually transfers across plume scale/noise changes.
        episode_scenario = args.scenario
        if args.mixed_domain and np.random.random() < args.transfer_fraction:
            episode_scenario = args.transfer_scenario
        cfg = make_training_config(episode_scenario, agents=args.agents,
                                   max_seconds=args.max_seconds,
                                   required_sources=args.required_sources,
                                   dynamic_source_fraction=args.dynamic_source_fraction,
                                   success_radius=args.success_radius)
        env.cfg = cfg
        env.coverage_progress_coef = args.coverage_progress_coef
        env.response_delta_coef = args.response_delta_coef
        env.diversity_coef = args.diversity_coef
        env.duplicate_coef = args.duplicate_coef
        # Multi-task curriculum: avoid letting the policy overfit to a fixed
        # early-stop target such as "confirm exactly three sources".
        if args.required_sources_min is not None:
            cfg.required_sources = int(np.random.randint(
                args.required_sources_min, args.required_sources_max + 1))
        if args.dynamic_source_fraction_min is not None:
            cfg.dynamic_source_fraction = float(np.random.uniform(
                args.dynamic_source_fraction_min, args.dynamic_source_fraction_max))
        obs = env.reset()
        hist, roles, recv, aux = obs
        traj = []
        done = False
        while not done:
            ht = torch.as_tensor(hist, dtype=torch.float32, device=device)
            rt = torch.as_tensor(roles, dtype=torch.long, device=device)
            gt = torch.as_tensor(recv, dtype=torch.float32, device=device)
            at = torch.as_tensor(aux, dtype=torch.float32, device=device)
            with torch.no_grad():
                actions, messages, alogp, mlogp, ent = actor.sample(ht, rt, gt, at)
                values = critic(ht.unsqueeze(0), onehot(rt).unsqueeze(0), gt.unsqueeze(0), at.unsqueeze(0))[0]
            teacher_actions, teacher_mask = env.coverage_teacher_actions()
            next_obs, reward, done, info = env.step(actions.cpu().numpy(), messages.cpu().numpy())
            traj.append((hist.copy(), roles.copy(), recv.copy(), aux.copy(),
                         actions.cpu().numpy(), messages.cpu().numpy(),
                         (alogp + mlogp).cpu().numpy(), float(values.cpu()),
                         reward.copy(), float(done), teacher_actions, teacher_mask))
            hist, roles, recv, aux = next_obs
        # Compute team returns and advantages.  Shared team reward keeps the
        # cooperative objective explicit while each actor remains decentralized.
        rewards = np.asarray([x[8].mean() for x in traj], dtype=np.float32)
        values = np.asarray([x[7] for x in traj] + [0.0], dtype=np.float32)
        adv = np.zeros(len(traj), dtype=np.float32)
        gae = 0.0
        for t in reversed(range(len(traj))):
            delta = rewards[t] + args.gamma * values[t + 1] - values[t]
            gae = delta + args.gamma * args.gae_lambda * gae
            adv[t] = gae
        returns = adv + values[:-1]
        # Flatten time and agents for shared actor update.
        H = torch.as_tensor(np.concatenate([x[0] for x in traj]), dtype=torch.float32, device=device)
        R = torch.as_tensor(np.concatenate([x[1] for x in traj]), dtype=torch.long, device=device)
        M = torch.as_tensor(np.concatenate([x[2] for x in traj]), dtype=torch.float32, device=device)
        A = torch.as_tensor(np.concatenate([x[3] for x in traj]), dtype=torch.float32, device=device)
        ACT = torch.as_tensor(np.concatenate([x[4] for x in traj]), dtype=torch.long, device=device)
        MSG = torch.as_tensor(np.concatenate([x[5] for x in traj]), dtype=torch.float32, device=device)
        old = torch.as_tensor(np.concatenate([x[6] for x in traj]), dtype=torch.float32, device=device)
        ADV = torch.as_tensor(np.repeat(adv, cfg.agents), dtype=torch.float32, device=device)
        RET = torch.as_tensor(np.repeat(returns, cfg.agents), dtype=torch.float32, device=device)
        ADV = (ADV - ADV.mean()) / (ADV.std() + 1e-8)
        for _ in range(args.epochs):
            new_a, new_m, entropy, logits = actor.evaluate(H, R, M, A, ACT, MSG)
            ratio = torch.exp(new_a + new_m - old)
            s1, s2 = ratio * ADV, ratio.clamp(1 - args.clip, 1 + args.clip) * ADV
            actor_loss = -torch.min(s1, s2).mean() - args.entropy_coef * entropy.mean()
            # Critic update on per-step centralized batches.
            idx = np.repeat(np.arange(len(traj)), cfg.agents)
            c_hist = H.reshape(len(traj), cfg.agents, 8, 21)
            c_roles = onehot(R.reshape(len(traj), cfg.agents))
            c_recv = M.reshape(len(traj), cfg.agents, MESSAGE_DIM)
            c_aux = A.reshape(len(traj), cfg.agents, 5)
            pred_v = critic(c_hist, c_roles, c_recv, c_aux)
            critic_loss = torch.nn.functional.mse_loss(pred_v, torch.as_tensor(returns, device=device))
            # Communication auxiliary objective: encode only policy-visible
            # local evidence (TGS response/difference/slope and odometry).
            # This prevents the message channel from collapsing to noise while
            # keeping source truth outside both actor and transmitted packet.
            latest = H[:, -1, :]
            # Structured semantic target for the learned packet: local plume
            # strength, left/right gradient, temporal slope, stable-plume
            # event, and sender pose/heading.  No source truth is used.
            msg_target = latest[:, [6, 5, 7, 9, 17, 18, 19, 20]].clamp(-1.0, 1.0)
            _, msg_mean, _ = actor.forward(H, R, M, A)
            communication_loss = torch.nn.functional.mse_loss(torch.tanh(msg_mean), msg_target)
            teacher = torch.as_tensor(np.concatenate([x[10] for x in traj]), dtype=torch.long, device=device)
            teacher_mask = torch.as_tensor(np.concatenate([x[11] for x in traj]), dtype=torch.bool, device=device)
            if teacher_mask.any():
                coverage_loss = torch.nn.functional.cross_entropy(logits[teacher_mask], teacher[teacher_mask])
            else:
                coverage_loss = torch.zeros((), device=device)
            loss = (actor_loss + args.value_coef * critic_loss
                    + args.communication_coef * communication_loss
                    + args.coverage_coef * coverage_loss)
            opt.zero_grad(); loss.backward(); torch.nn.utils.clip_grad_norm_(list(actor.parameters()) + list(critic.parameters()), 0.5); opt.step()
        row = {"episode": episode, "scenario": episode_scenario,
               "success": info["success"], "found": info["found"],
               "total": info["total"], "time_s": info["time_s"],
               "path_length": info["path_length"], "safety_overrides": info["safety_overrides"],
               "actor_loss": float(actor_loss.detach()), "critic_loss": float(critic_loss.detach()),
               "communication_loss": float(communication_loss.detach()),
               "coverage_loss": float(coverage_loss.detach()),
               "mean_reward": float(rewards.mean())}
        logs.append(row)
        if (episode + 1) % args.log_every == 0:
            print(json.dumps(row), flush=True)
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    torch.save({"actor": actor.state_dict(), "critic": critic.state_dict(),
                "config": vars(cfg), "algorithm": "hetero_mappo_v1",
                "message_dim": MESSAGE_DIM,
                "message_schema": "mean_diff_slope_stable_pose"}, args.checkpoint)
    out.write_text(json.dumps({"algorithm": "hetero_mappo_v1", "config": vars(cfg), "episodes": logs}, indent=2), encoding="utf-8")
    print(json.dumps({"checkpoint": args.checkpoint, "log": str(out), "episodes": len(logs),
                      "success_rate": float(np.mean([x["success"] for x in logs])),
                      "base_ppo_tensors_loaded": loaded}, indent=2))


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--scenario", default="competition_20m_10agents")
    p.add_argument("--mixed-domain", action="store_true",
                   help="sample normal and transfer scenarios at episode boundaries")
    p.add_argument("--transfer-scenario", default="transfer_hard_50m_10agents_20sources")
    p.add_argument("--transfer-fraction", type=float, default=0.5)
    p.add_argument("--agents", type=int, default=5)
    p.add_argument("--max-seconds", type=float, default=60.0)
    p.add_argument("--required-sources", type=int, default=None)
    p.add_argument("--dynamic-source-fraction", type=float, default=None)
    p.add_argument("--success-radius", type=float, default=None)
    p.add_argument("--required-sources-min", type=int, default=None)
    p.add_argument("--required-sources-max", type=int, default=10)
    p.add_argument("--dynamic-source-fraction-min", type=float, default=None)
    p.add_argument("--dynamic-source-fraction-max", type=float, default=0.3)
    p.add_argument("--episodes", type=int, default=100)
    p.add_argument("--epochs", type=int, default=4)
    p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--gamma", type=float, default=0.99)
    p.add_argument("--gae-lambda", type=float, default=0.95)
    p.add_argument("--clip", type=float, default=0.2)
    p.add_argument("--value-coef", type=float, default=0.5)
    p.add_argument("--entropy-coef", type=float, default=0.005)
    p.add_argument("--communication-coef", type=float, default=0.05)
    p.add_argument("--coverage-coef", type=float, default=0.05)
    p.add_argument("--packet-loss", type=float, default=0.0)
    p.add_argument("--message-aggregation", choices=("mean", "max_response"), default="mean")
    p.add_argument("--coverage-prior", action="store_true")
    p.add_argument("--coverage-pattern", choices=("radial", "radial_dense", "radial_slow", "radial_hard", "radial_wide", "radial_wide_hard", "radial_transfer", "lawnmower", "grid_dense", "raster_dense", "role_split"), default="radial")
    p.add_argument("--no-communication", action="store_true")
    p.add_argument("--start-layout", choices=("center", "ring"), default="center")
    p.add_argument("--confirmation-threshold-scale", type=float, default=1.0)
    p.add_argument("--confirmation-steps", type=int, default=2)
    p.add_argument("--coverage-progress-coef", type=float, default=0.02)
    p.add_argument("--response-delta-coef", type=float, default=0.04)
    p.add_argument("--diversity-coef", type=float, default=0.01)
    p.add_argument("--duplicate-coef", type=float, default=0.003)
    p.add_argument("--seed", type=int, default=20261102)
    p.add_argument("--device", default="cpu")
    p.add_argument("--log-every", type=int, default=10)
    p.add_argument("--checkpoint", default="MARL/new/artifacts/mappo.pt")
    p.add_argument("--output", default="MARL/new/artifacts/train.json")
    p.add_argument("--base-ppo", default=None)
    p.add_argument("--resume", default=None)
    train(p.parse_args())

