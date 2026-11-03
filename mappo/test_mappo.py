import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from env import LearnedMARLEnv, make_training_config
from mappo_core import HeteroMAPPOActor, CentralizedCritic


def test_shapes_and_forward():
    cfg = make_training_config("competition_20m_10agents", agents=5, max_seconds=2)
    env = LearnedMARLEnv(cfg, 7)
    h, r, m, a = env.reset()
    actor = HeteroMAPPOActor()
    critic = CentralizedCritic()
    with torch.no_grad():
        actions, messages, *_ = actor.sample(torch.tensor(h), torch.tensor(r), torch.tensor(m), torch.tensor(a))
        value = critic(torch.tensor(h).unsqueeze(0), torch.nn.functional.one_hot(torch.tensor(r), 4).float().unsqueeze(0), torch.tensor(m).unsqueeze(0), torch.tensor(a).unsqueeze(0))
    assert h.shape == (5, 8, 21)
    assert actions.shape == (5,)
    assert messages.shape == (5, 8)
    assert value.shape == (1,)


def test_episode_reproducible():
    cfg = make_training_config("small_2m_3agents", agents=2, max_seconds=2)
    a = LearnedMARLEnv(cfg, 123)
    b = LearnedMARLEnv(cfg, 123)
    assert np.array_equal(a.reset()[0], b.reset()[0])


if __name__ == "__main__":
    test_shapes_and_forward()
    test_episode_reproducible()
    print("mappo tests: ok")
