from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple

import torch
from torch import nn
from torch.distributions import Categorical, Normal


ROLE_COUNT = 4
MESSAGE_DIM = 8
ACTION_COUNT = 7


class HeteroMAPPOActor(nn.Module):
    """Shared actor with role conditioning and learned continuous messages.

    The motion head retains the existing seven-action PPO contract.  The
    message head is a small Gaussian policy; its sampled vector is relayed to
    neighbors on the next decision step.
    """

    def __init__(self, history_steps: int = 8, feature_dim: int = 21,
                 role_dim: int = 16, message_dim: int = MESSAGE_DIM):
        super().__init__()
        self.history_steps = history_steps
        self.feature_dim = feature_dim
        self.message_dim = message_dim
        self.obs_encoder = nn.Sequential(
            nn.Linear(history_steps * feature_dim, 128), nn.Tanh(),
            nn.Linear(128, 128), nn.Tanh(),
        )
        # Keep a baseline-compatible actor branch.  When a 21D single-agent
        # checkpoint is supplied, its trunk and actor weights can be loaded
        # here; the role/message branch starts as a zero residual.
        self.base_actor_fc = nn.Linear(128, 64)
        self.base_action_head = nn.Linear(64, ACTION_COUNT)
        self.role_embedding = nn.Embedding(ROLE_COUNT, role_dim)
        self.context = nn.Sequential(
            nn.Linear(128 + role_dim + message_dim + 5, 128),
            nn.Tanh(), nn.Linear(128, 128), nn.Tanh(),
        )
        self.action_head = nn.Linear(128, ACTION_COUNT)
        nn.init.zeros_(self.action_head.weight)
        nn.init.zeros_(self.action_head.bias)
        self.message_head = nn.Linear(128, message_dim)
        self.message_log_std = nn.Parameter(torch.full((message_dim,), -0.7))

    def forward(self, history, roles, received, aux):
        b = history.shape[0]
        z = self.obs_encoder(history.reshape(b, -1))
        r = self.role_embedding(roles)
        z = self.context(torch.cat([z, r, received, aux], dim=-1))
        base = self.base_action_head(torch.relu(self.base_actor_fc(
            self.obs_encoder(history.reshape(b, -1)))))
        return base + self.action_head(z), self.message_head(z), self.message_log_std.expand_as(self.message_head(z))

    def load_single_agent_checkpoint(self, path):
        """Load the compatible 21D PPO trunk/actor without touching critic."""
        checkpoint = torch.load(path, map_location="cpu", weights_only=False)
        state = checkpoint.get("network_state_dict") or checkpoint.get("model_state_dict")
        if state is None:
            raise RuntimeError("checkpoint has no network state")
        own = self.state_dict()
        mapping = {
            "fc_shared.0.weight": "obs_encoder.0.weight",
            "fc_shared.0.bias": "obs_encoder.0.bias",
            "fc_shared.2.weight": "obs_encoder.2.weight",
            "fc_shared.2.bias": "obs_encoder.2.bias",
            "actor_fc.weight": "base_actor_fc.weight",
            "actor_fc.bias": "base_actor_fc.bias",
            "actor_out.weight": "base_action_head.weight",
            "actor_out.bias": "base_action_head.bias",
        }
        copied = 0
        for src, dst in mapping.items():
            if src in state and own[dst].shape == state[src].shape:
                own[dst].copy_(state[src]); copied += 1
        self.load_state_dict(own)
        return copied

    def sample(self, history, roles, received, aux, deterministic=False):
        logits, msg_mean, msg_log_std = self.forward(history, roles, received, aux)
        action_dist = Categorical(logits=logits)
        msg_dist = Normal(msg_mean, msg_log_std.exp())
        action = logits.argmax(-1) if deterministic else action_dist.sample()
        raw_msg = msg_mean if deterministic else msg_dist.rsample()
        message = torch.tanh(raw_msg)
        action_logp = action_dist.log_prob(action)
        # Change-of-variables correction is not needed for the fixed bounded
        # protocol; using the Gaussian log-prob keeps the PPO objective stable.
        message_logp = msg_dist.log_prob(raw_msg).sum(-1)
        entropy = action_dist.entropy() + msg_dist.entropy().sum(-1)
        return action, message, action_logp, message_logp, entropy

    def evaluate(self, history, roles, received, aux, actions, messages):
        logits, msg_mean, msg_log_std = self.forward(history, roles, received, aux)
        action_dist = Categorical(logits=logits)
        msg_dist = Normal(msg_mean, msg_log_std.exp())
        # Stored messages are bounded; inverse tanh is numerically protected.
        clipped = messages.clamp(-0.999, 0.999)
        raw = 0.5 * (torch.log1p(clipped) - torch.log1p(-clipped))
        return (action_dist.log_prob(actions),
                msg_dist.log_prob(raw).sum(-1),
                action_dist.entropy() + msg_dist.entropy().sum(-1),
                logits)


class CentralizedCritic(nn.Module):
    """Permutation-invariant critic for variable agent counts."""

    def __init__(self, history_steps=8, feature_dim=21, message_dim=MESSAGE_DIM):
        super().__init__()
        self.agent_encoder = nn.Sequential(
            nn.Linear(history_steps * feature_dim + ROLE_COUNT + message_dim + 5, 128),
            nn.Tanh(), nn.Linear(128, 128), nn.Tanh(),
        )
        self.value_head = nn.Sequential(nn.Linear(128, 64), nn.Tanh(), nn.Linear(64, 1))

    def forward(self, histories, role_onehot, received, aux):
        # [B,N,...] -> [B,N,128], then mean pooling over agents.
        b, n = histories.shape[:2]
        x = torch.cat([
            histories.reshape(b, n, -1), role_onehot, received, aux
        ], dim=-1)
        z = self.agent_encoder(x.reshape(b * n, -1)).reshape(b, n, -1)
        return self.value_head(z.mean(dim=1)).squeeze(-1)


@dataclass
class PPOBatch:
    histories: torch.Tensor
    roles: torch.Tensor
    role_onehot: torch.Tensor
    received: torch.Tensor
    aux: torch.Tensor
    actions: torch.Tensor
    messages: torch.Tensor
    old_logp: torch.Tensor
    returns: torch.Tensor
    advantages: torch.Tensor
    values: torch.Tensor

