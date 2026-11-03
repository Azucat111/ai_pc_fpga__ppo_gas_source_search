#!/usr/bin/env python3
"""Evaluate PPO transfer to larger maps and wider source distributions."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from config.config import EnvironmentConfig
from environment.robot_env_v3 import OdorSourceLocalizationEnvV3
from models.ppo_network import PPOAgent


class EvalConfig:
    DEVICE = 'cpu'
    LEARNING_RATE = 3e-4
    PPO_EPOCHS = 1
    CLIP_EPSILON = 0.2
    VALUE_COEF = 0.5
    ENTROPY_COEF = 0.01
    MAX_GRAD_NORM = 0.5


def evaluate(agent, name, width, height, source_max, episodes, seed_offset,
             max_episode_seconds):
    config = EnvironmentConfig()
    config.INCLUDE_GAS_RESISTANCE = False
    config.INCLUDE_POSITION = True
    config.FIELD_SIZE_RANGE = ((width, width), (height, height))
    config.SOURCE_CENTER_DISTANCE_RANGE = (0.5, source_max)
    config.MAX_EPISODE_SECONDS = float(max_episode_seconds)
    successes = collisions = timeouts = overrides = 0
    success_steps = []
    for episode in range(episodes):
        env = OdorSourceLocalizationEnvV3(config)
        state = env.reset(random_seed=seed_offset + episode)
        for step in range(env.max_steps):
            with torch.no_grad():
                action = agent.select_action(state, training=False)
            state, _, done, info = env.step(action)
            overrides += int(info.get('safety_override', False))
            if done:
                if info.get('success'):
                    successes += 1
                    success_steps.append(step + 1)
                elif info.get('collision'):
                    collisions += 1
                else:
                    timeouts += 1
                break
    return {
        'scenario': name,
        'map_m': [width, height],
        'area_m2': width * height,
        'source_center_radius_m': [0.5, source_max],
        'max_episode_seconds': max_episode_seconds,
        'episodes': episodes,
        'success_rate_percent': successes / episodes * 100.0,
        'successes': successes,
        'collisions': collisions,
        'timeouts': timeouts,
        'mean_success_steps': float(np.mean(success_steps)) if success_steps else None,
        'safety_overrides': overrides,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--episodes', type=int, default=200)
    parser.add_argument('--seed-offset', type=int, default=700000)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--max-episode-seconds', type=float, default=60.0)
    args = parser.parse_args()
    if args.episodes < 1:
        parser.error('--episodes must be positive')
    if args.max_episode_seconds <= 0:
        parser.error('--max-episode-seconds must be positive')
    agent = PPOAgent((8, 21), 7, EvalConfig)
    agent.load(str(args.checkpoint))
    agent.network.eval()
    cases = (
        ('training_scale_9m2', 3.0, 3.0, 1.2),
        ('large_50m2_center_near_source', 5.0, 10.0, 1.2),
        ('large_100m2_center_near_source', 10.0, 10.0, 1.2),
        ('large_50m2_full_area_source', 5.0, 10.0, 4.5),
        ('large_100m2_full_area_source', 10.0, 10.0, 6.5),
    )
    results = [evaluate(agent, name, width, height, source_max,
                        args.episodes, args.seed_offset,
                        args.max_episode_seconds)
               for name, width, height, source_max in cases]
    text = json.dumps(results, ensure_ascii=False, indent=2) + '\n'
    if args.output:
        args.output.write_text(text, encoding='utf-8')
    print(text, end='')


if __name__ == '__main__':
    main()
