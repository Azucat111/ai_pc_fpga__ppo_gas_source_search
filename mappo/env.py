from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

BASE = Path(__file__).resolve().parents[1]
if str(BASE) not in sys.path:
    sys.path.insert(0, str(BASE))

from marl.actions import ACTION_SPEED, STOP
from marl.config import SimulationConfig, scenario_config
from marl.observations import ObservationHistory, make_observation
from marl.plume import OdorField


ROLES = ("scout", "tracker", "confirmer", "relay")
ROLE_CAPS = {
    "scout": (1.25, 0.90, 0.60, 0.80),
    "tracker": (0.90, 1.20, 0.50, 1.00),
    "confirmer": (0.70, 1.05, 0.42, 1.00),
    "relay": (0.80, 0.85, 0.65, 1.80),
}


@dataclass
class AgentState:
    agent_id: int
    position: np.ndarray
    heading: float
    history: ObservationHistory
    role: str
    speed_scale: float
    sensor_gain: float
    confirmation_threshold: float
    communication_range_scale: float
    last_action: int = STOP
    action_age: float = 0.0
    last_linear: float = 0.0
    last_angular: float = 0.0
    last_sensor: object = None
    plume_stable: bool = False
    path_length: float = 0.0
    confirmed_streak: int = 0
    confirmed: bool = False


class LearnedMARLEnv:
    """Small on-policy environment using the baseline plume/sensor model."""

    def __init__(self, cfg: SimulationConfig, seed: int, communication=True,
                 packet_loss=0.0, message_aggregation="mean", coverage_prior=False,
                 coverage_pattern="radial", start_layout="center",
                 coverage_progress_coef=0.02, response_delta_coef=0.04,
                 diversity_coef=0.01, duplicate_coef=0.003,
                 confirmation_threshold_scale=1.0, confirmation_steps=2):
        self.cfg = cfg
        self.rng = np.random.default_rng(seed)
        self.communication = communication
        self.packet_loss = packet_loss
        self.message_aggregation = message_aggregation
        self.coverage_prior = coverage_prior
        self.coverage_pattern = coverage_pattern
        self.start_layout = start_layout
        self.coverage_progress_coef = coverage_progress_coef
        self.response_delta_coef = response_delta_coef
        self.diversity_coef = diversity_coef
        self.duplicate_coef = duplicate_coef
        self.confirmation_threshold_scale = confirmation_threshold_scale
        self.confirmation_steps = max(1, int(confirmation_steps))
        self.roles = [ROLES[i % len(ROLES)] for i in range(cfg.agents)]
        self.reset()

    @property
    def agent_count(self):
        return self.cfg.agents

    def reset(self):
        c = np.array([self.cfg.width / 2, self.cfg.height / 2], dtype=float)
        self.field = OdorField(
            self.cfg.width, self.cfg.height, self.rng,
            (self.cfg.wind_x, self.cfg.wind_y), self.cfg.source_min_center_distance,
            self.cfg.source_max_center_distance, self.cfg.source_boundary_margin,
            self.cfg.plume_spread, self.cfg.sensor_offset, self.cfg.source_count,
            self.cfg.source_min_separation, self.cfg.source_layout,
            self.cfg.dynamic_source_fraction, self.cfg.dynamic_appearance_min_s,
            self.cfg.dynamic_appearance_max_s, self.cfg.dynamic_source_layout)
        self.agents = []
        for i in range(self.cfg.agents):
            role = self.roles[i]
            speed, gain, threshold, comm = ROLE_CAPS[role]
            if self.start_layout == "ring":
                radius = min(self.cfg.width, self.cfg.height) * 0.30
                angle = 2.0 * math.pi * i / max(self.cfg.agents, 1)
                position = c + radius * np.array([math.cos(angle), math.sin(angle)])
                position = np.clip(position, [0.0, 0.0], [self.cfg.width, self.cfg.height])
                heading = (angle + math.pi + math.pi) % (2.0 * math.pi) - math.pi
            else:
                position = c.copy()
                heading = self.rng.uniform(-math.pi, math.pi)
            self.agents.append(AgentState(i, position, heading,
                                          ObservationHistory(), role, speed, gain,
                                          threshold * self.confirmation_threshold_scale, comm))
        self.time_s = 0.0
        self.found = set()
        self.last_found = 0
        self.source_min_distance = np.full(self.cfg.source_count, np.inf, dtype=np.float32)
        self.prev_nearest = None
        self.received = np.zeros((self.cfg.agents, 8), dtype=np.float32)
        self.message_age = np.full(self.cfg.agents, 1.0, dtype=np.float32)
        self.done = False
        # Seed the fixed eight-step history with a real local sensor sample;
        # an empty history is not a valid input to the existing PPO contract.
        for a in self.agents:
            a.history.append(make_observation(a, self._sensor(a), self.cfg))
        return self.observe()

    def _sensor(self, a):
        x = self.field.sample(a.position, a.heading, self.rng, self.cfg.sensor_noise, self.time_s)
        return np.clip(x * a.sensor_gain, 0.0, 1.0)

    def _aux(self):
        # communication quality, age, normalized local speed, angular speed,
        # and role-independent sensor validity summary.
        out = []
        for i, a in enumerate(self.agents):
            quality = 0.0 if not self.communication else max(0.0, 1.0 - self.message_age[i] / 5.0)
            out.append([quality, min(self.message_age[i], 5.0) / 5.0,
                        a.last_linear / 0.2, a.last_angular / 1.2,
                        float(a.last_sensor is not None)])
        return np.asarray(out, dtype=np.float32)

    def observe(self):
        histories = np.asarray([a.history.array() for a in self.agents], dtype=np.float32)
        roles = np.asarray([ROLES.index(a.role) for a in self.agents], dtype=np.int64)
        return histories, roles, self.received.copy(), self._aux()

    def coverage_teacher_actions(self):
        """Source-blind sector coverage labels for an auxiliary actor loss.

        The target uses only role/ID, pose and elapsed task time.  It never
        reads source locations or plume truth, so it is a deployable coverage
        prior rather than privileged source supervision.
        """
        radii = (0.22, 0.38, 0.50)
        phase = int(self.time_s / max(getattr(self.cfg, "coverage_period_s", 75.0), 1.0))
        center = np.array([self.cfg.width / 2.0, self.cfg.height / 2.0])
        radius = min(self.cfg.width, self.cfg.height) * radii[phase % len(radii)]
        labels, mask = [], []
        for a in self.agents:
            angle = (2.0 * math.pi * a.agent_id / max(self.cfg.agents, 1) + phase * 0.55) % (2.0 * math.pi)
            target = center + radius * np.array([math.cos(angle), math.sin(angle)])
            target_angle = math.atan2(*(target - a.position)[::-1])
            error = (target_angle - a.heading + math.pi) % (2 * math.pi) - math.pi
            if abs(error) > 0.35:
                action = 3 if error > 0 else 4
            else:
                action = 2
            response = float(a.last_sensor[2]) if a.last_sensor is not None else 0.0
            labels.append(action)
            mask.append(response < self.cfg.coverage_response_threshold)
        return np.asarray(labels, dtype=np.int64), np.asarray(mask, dtype=bool)

    def _deliver(self, messages):
        new_received = np.zeros_like(self.received)
        new_age = np.full_like(self.message_age, 1.0)
        if not self.communication:
            self.received, self.message_age = new_received, new_age
            return
        for i, target in enumerate(self.agents):
            candidate_indices = []
            for j, sender in enumerate(self.agents):
                if i == j:
                    continue
                r = self.cfg.communication_range * min(target.communication_range_scale,
                                                       sender.communication_range_scale)
                if np.linalg.norm(target.position - sender.position) <= r and self.rng.random() >= self.packet_loss:
                    candidate_indices.append(j)
            if candidate_indices:
                if self.message_aggregation == "max_response":
                    # The packet is still learned, but the relay preserves the
                    # strongest local evidence instead of averaging it away.
                    best = max(candidate_indices, key=lambda j: float(self.agents[j].last_sensor[2])
                                   if self.agents[j].last_sensor is not None else 0.0)
                    new_received[i] = messages[best]
                else:
                    new_received[i] = np.mean(messages[candidate_indices], axis=0)
                new_age[i] = 0.0
        self.received, self.message_age = new_received, new_age

    def step(self, actions, messages):
        old_dist = self._coverage_cost()
        previous_response = np.asarray([
            float(a.last_sensor[2]) if a.last_sensor is not None else 0.0
            for a in self.agents
        ], dtype=np.float32)
        safety = 0
        coverage_overrides = 0
        for a, action in zip(self.agents, actions):
            action = int(action)
            hard_coverage = (self.coverage_pattern in ("radial_hard", "radial_wide_hard", "raster_dense") and
                             (a.last_sensor is None or
                              float(a.last_sensor[2]) < self.cfg.tracking_response_threshold))
            role_coverage = (self.coverage_pattern == "role_split" and a.role in ("scout", "relay") and
                             (a.last_sensor is None or
                              float(a.last_sensor[2]) < self.cfg.tracking_response_threshold))
            if self.coverage_prior and (self._low_response(a) or hard_coverage or role_coverage):
                prior = self._coverage_action(a)
                if prior != action:
                    coverage_overrides += 1
                action = prior
            v, omega = ACTION_SPEED[action]
            next_heading = a.heading + omega * self.cfg.dt
            margin = max(0.12, min(self.cfg.width, self.cfg.height) * 0.02)
            predicted = a.position + v * self.cfg.motion_scale * self.cfg.dt * np.array([
                math.cos(next_heading), math.sin(next_heading)])
            if (predicted[0] < margin or predicted[0] > self.cfg.width - margin or
                    predicted[1] < margin or predicted[1] > self.cfg.height - margin):
                safety += 1
                action = 3 if self.cfg.width / 2 - a.position[0] >= 0 else 4
                v, omega = ACTION_SPEED[action]
            a.heading = (a.heading + omega * self.cfg.dt + math.pi) % (2 * math.pi) - math.pi
            delta = v * self.cfg.motion_scale * a.speed_scale * self.cfg.dt * np.array([
                math.cos(a.heading), math.sin(a.heading)])
            nxt = np.clip(a.position + delta, [0.0, 0.0], [self.cfg.width, self.cfg.height])
            a.path_length += float(np.linalg.norm(nxt - a.position))
            a.position = nxt
            a.last_action, a.action_age = action, 0.0
            a.last_linear, a.last_angular = v, omega
        self._deliver(np.asarray(messages, dtype=np.float32))
        self.time_s += self.cfg.dt
        self.field.update(self.time_s)
        positions_now = np.asarray([a.position for a in self.agents])
        for idx, source in enumerate(self.field.sources):
            if source is not None:
                self.source_min_distance[idx] = min(
                    float(self.source_min_distance[idx]),
                    float(np.min(np.linalg.norm(positions_now - source, axis=1))))
        for a in self.agents:
            sensor = self._sensor(a)
            obs = make_observation(a, sensor, self.cfg)
            if a.last_sensor is not None and sensor[2] >= a.confirmation_threshold:
                a.confirmed_streak += 1
            else:
                a.confirmed_streak = max(0, a.confirmed_streak - 1)
            a.confirmed = a.confirmed_streak >= self.confirmation_steps
            a.history.append(obs)
        for idx, source in enumerate(self.field.sources):
            if source is None or not self.field.is_active(idx, self.time_s):
                continue
            if any(np.linalg.norm(a.position - source) < self.cfg.success_radius and a.confirmed for a in self.agents):
                self.found.add(idx)
        new_dist = self._coverage_cost()
        progress = 0.0 if old_dist is None or new_dist is None else (old_dist - new_dist)
        new_found = len(self.found)
        current_response = np.asarray([
            float(a.last_sensor[2]) if a.last_sensor is not None else 0.0
            for a in self.agents
        ], dtype=np.float32)
        response_delta = np.clip(current_response - previous_response, -0.2, 0.2)
        # Encourage local plume acquisition while retaining the centralized
        # source-progress term.  This is observable through the local sensor
        # and therefore does not leak source truth into the actor.
        reward = (self.coverage_progress_coef * progress + self.response_delta_coef * response_delta - 0.002
                  - 0.0005 * safety).astype(np.float32)
        # A mild diversity term makes multi-source coverage preferable to ten
        # vehicles collapsing onto the same plume.  It is bounded and has no
        # effect on the safety layer.
        if len(self.agents) > 1:
            positions = np.asarray([a.position for a in self.agents])
            pairwise = np.linalg.norm(positions[:, None, :] - positions[None, :, :], axis=-1)
            nearest = np.min(np.where(np.eye(len(self.agents), dtype=bool), np.inf, pairwise), axis=1)
            diversity = np.clip(nearest / max(min(self.cfg.width, self.cfg.height), 1.0), 0.0, 1.0)
            duplicate = (nearest < max(self.cfg.success_radius * 0.5, 0.25)).astype(np.float32)
            reward += self.diversity_coef * diversity - self.duplicate_coef * duplicate
        if new_found > getattr(self, "last_found", 0):
            reward += 1.0 * (new_found - getattr(self, "last_found", 0))
        self.last_found = new_found
        done = len(self.found) >= self.cfg.required_sources or self.time_s >= self.cfg.max_seconds
        self.done = done
        info = {"time_s": self.time_s, "found": len(self.found), "total": len(self.field.sources),
                "success": len(self.found) >= self.cfg.required_sources,
                "safety_overrides": safety, "path_length": sum(a.path_length for a in self.agents),
                "communication": self.communication, "coverage_overrides": coverage_overrides,
                # Diagnostics only: never included in actor observations or
                # reward.  These fields identify which source instances are
                # missed so transfer failures can be debugged by layout/type.
                "found_indices": sorted(int(i) for i in self.found),
                "source_diagnostics": [
                    None if source is None else {
                        "index": int(i),
                        "x": float(source[0]), "y": float(source[1]),
                        "active": bool(self.field.is_active(i, self.time_s)),
                        "dynamic": bool(i in self.field.dynamic_indices),
                        "radius_from_center": float(np.linalg.norm(
                            source - np.array([self.cfg.width / 2.0, self.cfg.height / 2.0]))),
                        "min_agent_distance": float(self.source_min_distance[i])
                    }
                    for i, source in enumerate(self.field.sources)
                ]}
        return self.observe(), reward, done, info

    def _low_response(self, agent):
        return agent.last_sensor is None or float(agent.last_sensor[2]) < self.cfg.coverage_response_threshold

    def _coverage_action(self, agent):
        phase = int(self.time_s / max(getattr(self.cfg, "coverage_period_s", 75.0), 1.0))
        if self.coverage_pattern in ("lawnmower", "grid_dense", "raster_dense"):
            # Large-map transfer: source-blind lane sweep gives more uniform
            # coverage than repeatedly cycling three radial rings.
            if self.coverage_pattern == "raster_dense":
                # Twenty lanes for ten agents; each phase advances an agent
                # to its neighboring lane and alternates the sweep endpoint.
                # A 300s period matches the simulated 0.2 m/s travel time
                # across the 50m map instead of repeatedly turning early.
                lanes = max(2 * self.cfg.agents, 1)
                lane_phase = int(self.time_s / 300.0)
                lane = (2 * agent.agent_id + lane_phase) % lanes
                lane_x = self.cfg.width * (lane + 0.5) / lanes
                target_y = self.cfg.height * (0.08 if lane_phase % 2 == 0 else 0.92)
            else:
                lane_x = self.cfg.width * (agent.agent_id + 0.5) / max(self.cfg.agents, 1)
            if self.coverage_pattern == "grid_dense":
                # Three y-levels reduce the blind strip left by the original
                # two-endpoint lawnmower while remaining source-blind.
                y_levels = (0.08, 0.50, 0.92)
                target_y = self.cfg.height * y_levels[phase % len(y_levels)]
            elif self.coverage_pattern == "lawnmower":
                target_y = self.cfg.height * (0.10 if phase % 2 == 0 else 0.90)
            target = np.array([lane_x, target_y])
            target_angle = math.atan2(*(target - agent.position)[::-1])
            error = (target_angle - agent.heading + math.pi) % (2 * math.pi) - math.pi
            if abs(error) > 0.35:
                return 3 if error > 0 else 4
            return 2
        if self.coverage_pattern == "radial_transfer":
            radii = (0.16, 0.24, 0.32, 0.40, 0.48, 0.56)
            phase = int(self.time_s / 45.0)
            phase_angle = phase * 0.30
        elif self.coverage_pattern in ("radial_dense", "radial_slow", "radial_hard", "radial_wide", "radial_wide_hard", "role_split"):
            if self.coverage_pattern in ("radial_wide", "radial_wide_hard"):
                # Reach the outer source band in the 50m transfer map.  The
                # target remains source-blind and is clipped by the safety
                # layer when it approaches the boundary.
                radii = (0.16, 0.30, 0.44, 0.58, 0.72, 0.86, 0.94)
                period = 55.0
                angle_step = 0.28
            else:
                radii = (0.14, 0.26, 0.38, 0.50, 0.62, 0.74)
                period = (75.0 if self.coverage_pattern == "radial_hard" else
                          (100.0 if self.coverage_pattern == "radial_slow" else 50.0))
                angle_step = (0.15 if self.coverage_pattern == "radial_hard" else
                              (0.20 if self.coverage_pattern == "radial_slow" else 0.35))
            phase = int(self.time_s / period)
            phase_angle = phase * angle_step
        else:
            radii = (0.22, 0.38, 0.50)
            phase_angle = phase * 0.55
        center = np.array([self.cfg.width / 2.0, self.cfg.height / 2.0])
        radius = min(self.cfg.width, self.cfg.height) * radii[phase % len(radii)]
        angle = (2.0 * math.pi * agent.agent_id / max(self.cfg.agents, 1) + phase_angle) % (2.0 * math.pi)
        target = center + radius * np.array([math.cos(angle), math.sin(angle)])
        target_angle = math.atan2(*(target - agent.position)[::-1])
        error = (target_angle - agent.heading + math.pi) % (2 * math.pi) - math.pi
        if abs(error) > 0.35:
            return 3 if error > 0 else 4
        return 2

    def _coverage_cost(self):
        """Mean distance from each active unfound source to its nearest car.

        The previous minimum-over-all-pairs potential rewarded every vehicle
        for chasing the same source.  This team coverage potential is still a
        training-only scalar (the actor never observes source truth), but it
        gives MAPPO a usable credit signal for multi-source allocation.
        """
        active = [s for i, s in enumerate(self.field.sources)
                  if s is not None and i not in self.found and self.field.is_active(i, self.time_s)]
        if not active:
            return None
        positions = np.asarray([a.position for a in self.agents])
        distances = [np.min(np.linalg.norm(positions - source, axis=1)) for source in active]
        return float(np.mean(distances) / max(np.hypot(self.cfg.width, self.cfg.height), 1.0))


def make_training_config(name="competition_20m_10agents", agents=None, max_seconds=None,
                         required_sources=None, dynamic_source_fraction=None,
                         success_radius=None):
    cfg = scenario_config(name, agents=agents, max_seconds=max_seconds,
                          required_sources=required_sources,
                          dynamic_source_fraction=dynamic_source_fraction,
                          success_radius=success_radius)
    return cfg

