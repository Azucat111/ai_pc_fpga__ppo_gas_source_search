from collections import deque
import numpy as np


FEATURE_NAMES = (
    "alcohol_prob", "temperature", "humidity", "tgs_left_norm", "tgs_right_norm",
    "tgs_diff_norm", "tgs_mean", "tgs_slope", "plume_enter", "plume_stable",
    "plume_lost", "linear_velocity", "angular_velocity", "gyro_z", "last_action",
    "action_age", "communication_quality", "x_norm", "y_norm", "sin_heading", "cos_heading",
)


class ObservationHistory:
    def __init__(self, size=8):
        self.values = deque(maxlen=size)

    def append(self, value):
        value = np.asarray(value, dtype=np.float32)
        self.values.append(value)
        while len(self.values) < self.values.maxlen:
            self.values.appendleft(value.copy())

    def array(self):
        return np.asarray(self.values, dtype=np.float32)


def make_observation(agent, sensor, cfg):
    left, right, mean = sensor
    diff = left - right
    previous = agent.last_sensor[2] if agent.last_sensor is not None else mean
    slope = (mean - previous) / cfg.dt
    stable = float(mean > 0.30)
    enter = float(stable and not agent.plume_stable)
    lost = float((not stable) and agent.plume_stable)
    agent.plume_stable = bool(stable)
    alcohol = float(np.clip(mean * 1.8, 0.0, 1.0))
    obs = np.array([
        alcohol, 0.0, 0.0, left, right, diff, mean, slope,
        enter, stable, lost, agent.last_linear, agent.last_angular,
        agent.last_angular, float(agent.last_action), agent.action_age, 1.0,
        agent.position[0] / cfg.width, agent.position[1] / cfg.height,
        np.sin(agent.heading), np.cos(agent.heading),
    ], dtype=np.float32)
    agent.last_sensor = sensor
    return obs
