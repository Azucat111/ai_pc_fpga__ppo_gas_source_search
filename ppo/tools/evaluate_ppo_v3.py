#!/usr/bin/env python3
"""独立评估PPO v3确定性策略。"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from config.config import EnvironmentConfig
from environment.robot_env_v3 import OdorSourceLocalizationEnvV3
from models.ppo_network import PPOAgent, PPOPositionResidualNetwork


class EvalConfig:
    DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'
    LEARNING_RATE = 3e-4
    PPO_EPOCHS = 1
    CLIP_EPSILON = 0.2
    VALUE_COEF = 0.5
    ENTROPY_COEF = 0.01
    MAX_GRAD_NORM = 0.5


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--checkpoint', required=True)
    parser.add_argument('--feature-count', type=int, choices=(17, 18, 21, 22), default=21)
    parser.add_argument('--episodes', type=int, default=1000)
    parser.add_argument('--seed-offset', type=int, default=0)
    parser.add_argument('--no-boundary-safety', action='store_true')
    args = parser.parse_args()

    checkpoint = torch.load(args.checkpoint, map_location='cpu', weights_only=False)
    residual = checkpoint.get('architecture') == 'position_residual_v1'
    if residual:
        network = PPOPositionResidualNetwork((8, 21), 7)
        network.load_state_dict(checkpoint['network_state_dict'])
        network.eval()
        def select_action(state):
            with torch.no_grad():
                logits, _ = network(torch.from_numpy(state).float().unsqueeze(0))
            return int(logits.argmax(dim=-1).item())
    else:
        agent = PPOAgent((8, args.feature_count), 7, EvalConfig)
        agent.load(args.checkpoint)
        agent.network.eval()
        def select_action(state):
            return agent.select_action(state, training=False)
    env_config = EnvironmentConfig()
    env_config.INCLUDE_GAS_RESISTANCE = args.feature_count in (18, 22)
    env_config.INCLUDE_POSITION = args.feature_count in (21, 22)
    env_config.BOUNDARY_SAFETY_ENABLED = not args.no_boundary_safety

    success = 0
    success_steps = []
    terminations = {}
    safety_overrides = 0
    for index in range(args.episodes):
        env = OdorSourceLocalizationEnvV3(env_config)
        state = env.reset(random_seed=args.seed_offset + index)
        for step in range(env.max_steps):
            action = select_action(state)
            state, _, done, info = env.step(action)
            safety_overrides += int(info.get('safety_override', False))
            if done:
                if info['success']:
                    success += 1
                    success_steps.append(step + 1)
                    key = 'success'
                elif info['collision']:
                    key = 'collision'
                else:
                    key = 'timeout'
                terminations[key] = terminations.get(key, 0) + 1
                break

    result = {
        'checkpoint': str(Path(args.checkpoint)),
        'episodes': args.episodes,
        'seed_offset': args.seed_offset,
        'success_rate_percent': success / max(args.episodes, 1) * 100.0,
        'mean_success_steps': float(np.mean(success_steps)) if success_steps else None,
        'terminations': terminations,
        'safety_overrides': safety_overrides,
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
