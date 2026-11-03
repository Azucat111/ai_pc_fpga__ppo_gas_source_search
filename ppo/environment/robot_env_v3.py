"""
气味源定位仿真环境 - v3版本：密集奖励
核心改进：
1. 距离引导奖励（每步都有反馈）
2. 浓度梯度奖励（鼓励上升）
3. Plume检测大奖励
4. 持续接近奖励
"""

import numpy as np
from collections import deque
from .plume_model import GaussianPlumeModel, DualTGS2602Sensor
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from config.config import RewardConfig


class OdorSourceLocalizationEnvV3:
    """
    v3环境 - 密集奖励版本

    关键改进：
    - 每步都有距离反馈
    - 浓度增加给正奖励
    - Plume事件大幅奖励
    """

    def __init__(self, config):
        """初始化环境"""
        self.config = config
        # 一个策略动作持续一个决策周期；传感器在该周期内按 config.DT 更新。
        self.sensor_dt = config.DT
        self.dt = 1.0 / config.DECISION_FREQ

        # 动作空间
        self.VELOCITY_MAP = config.VELOCITY_MAP
        self.ACTIONS = list(config.VELOCITY_MAP.keys())

        # 场地边界
        self.field_width, self.field_height = config.FIELD_SIZE
        self.robot_radius = config.ROBOT_RADIUS

        # 成功判定
        self.success_radius = config.SUCCESS_RADIUS
        self.success_threshold = config.SUCCESS_THRESHOLD
        self.success_window = config.SUCCESS_WINDOW
        self.max_steps = int(round(config.MAX_EPISODE_SECONDS * config.DECISION_FREQ))
        self.rng = np.random.default_rng()

        # 状态历史
        self.state_history = deque(maxlen=8)

        # 环境状态
        self.source_pos = None
        self.robot_pos = None
        self.robot_yaw = None
        self.initial_yaw = 0.0
        self.robot_vel = None
        self.plume_model = None
        self.dual_sensor = None

        # Episode信息
        self.steps = 0
        self.last_action = 0
        self.consecutive_stable_steps = 0
        self.episode_reward = 0.0

        # v3新增：用于密集奖励计算
        self.prev_distance = None  # 前一步距离
        self.prev_concentration = 0.0  # 前一步浓度
        self.prev_alcohol_prob = 0.0
        self.prev_tgs_mean = 0.0
        self.min_distance = float('inf')  # 历史最小距离

    @property
    def observation_space(self):
        """观测空间: (8, 18)或(8, 17)，不包含障碍物距离。"""
        class Space:
            def __init__(self):
                feature_count = (18 if self_config.INCLUDE_GAS_RESISTANCE else 17)
                if self_config.INCLUDE_POSITION:
                    feature_count += 4
                self.shape = (8, feature_count)
        self_config = self.config
        return Space()

    @property
    def action_space(self):
        """动作空间"""
        class Space:
            def __init__(self, n):
                self.n = n
        return Space(len(self.ACTIONS))

    def reset(self, random_seed=None):
        """重置环境"""
        if random_seed is not None:
            self.rng = np.random.default_rng(random_seed)

        # 每回合随机矩形地图，小车固定在地图中心，气源在合法环带内随机。
        (w_min, w_max), (h_min, h_max) = self.config.FIELD_SIZE_RANGE
        self.field_width = float(self.rng.uniform(w_min, w_max))
        self.field_height = float(self.rng.uniform(h_min, h_max))
        center = np.array([self.field_width / 2.0, self.field_height / 2.0])
        self.robot_pos = center.copy()
        self.robot_yaw = (self.rng.uniform(0, 2 * np.pi)
                          if self.config.INITIAL_YAW_RANDOM else 0.0)
        self.initial_yaw = self.robot_yaw

        min_source_margin = max(self.config.SOURCE_BOUNDARY_MARGIN,
                                 self.robot_radius)
        min_center_dist, max_center_dist = self.config.SOURCE_CENTER_DISTANCE_RANGE
        for _ in range(1000):
            angle = self.rng.uniform(0, 2 * np.pi)
            radius = self.rng.uniform(min_center_dist, max_center_dist)
            candidate = center + radius * np.array([np.cos(angle), np.sin(angle)])
            if (min_source_margin <= candidate[0] <= self.field_width - min_source_margin and
                    min_source_margin <= candidate[1] <= self.field_height - min_source_margin):
                self.source_pos = candidate
                break
        else:
            raise RuntimeError("无法在当前地图尺寸下采样合法气源位置")
        self.robot_vel = np.array([0.0, 0.0])

        # 初始化传感器和羽流
        self.plume_model = GaussianPlumeModel(
            self.source_pos,
            diffusion=self.config.PLUME_DIFFUSION,
            decay=self.config.PLUME_DECAY,
            base_width=self.config.PLUME_WIDTH,
            scaling=self.config.PLUME_SCALING,
        )
        self.dual_sensor = DualTGS2602Sensor(
            tau_response=self.config.TGS2602_TAU,
            tau_recovery=self.config.TGS2602_RECOVERY_TAU,
            gain_left=float(self.rng.normal(1.0, 0.03)),
            gain_right=float(self.rng.normal(1.0, 0.03)),
            noise_std=self.config.SENSOR_NOISE_STD,
        )

        # 重置统计
        self.steps = 0
        self.last_action = 0
        self.consecutive_stable_steps = 0
        self.episode_reward = 0.0

        # v3: 初始化距离和浓度
        self.prev_distance = np.linalg.norm(self.robot_pos - self.source_pos)
        self.prev_concentration = self.plume_model.sample(self.robot_pos)
        self.min_distance = self.prev_distance

        self.prev_alcohol_prob = 0.0
        self.prev_tgs_mean = 0.0

        # 初始化状态历史
        self.state_history.clear()
        initial_obs = self._get_observation()
        for _ in range(8):
            self.state_history.append(self._obs_to_vec(initial_obs))

        return self._get_stacked_state()

    def step(self, action):
        """执行一步"""
        self.steps += 1
        requested_action = action
        if self.config.INCLUDE_POSITION and self.config.BOUNDARY_SAFETY_ENABLED:
            action = self._boundary_safe_action(action)
        self.last_action = action

        # 获取动作对应的速度
        if not 0 <= action < len(self.ACTIONS):
            raise ValueError(f"动作越界: {action}")
        v, omega = self.VELOCITY_MAP[self.ACTIONS[action]]
        self.robot_vel = np.array([v, omega])

        # 运动学更新
        self.robot_yaw += omega * self.dt
        self.robot_yaw = np.arctan2(np.sin(self.robot_yaw), np.cos(self.robot_yaw))

        dx = v * np.cos(self.robot_yaw) * self.dt
        dy = v * np.sin(self.robot_yaw) * self.dt
        self.robot_pos += np.array([dx, dy])

        # 获取观测
        obs = self._get_observation()
        self.state_history.append(self._obs_to_vec(obs))

        # v3密集奖励
        reward = self._compute_reward_v3(obs, action)

        # 检查终止
        done, info = self._check_done(obs, requested_action, action)

        # 成功大奖励
        if info['success']:
            reward += 500.0

        self.episode_reward += reward

        # 更新历史
        self.prev_alcohol_prob = obs['alcohol_prob']
        self.prev_tgs_mean = obs['tgs_mean']
        self.prev_distance = obs['distance_to_source']
        self.prev_concentration = self.plume_model.sample(self.robot_pos)

        return self._get_stacked_state(), reward, done, info

    def _compute_reward_v3(self, obs, action):
        """
        v3密集奖励函数

        核心思想：每步都有明确的方向指导
        """
        reward = 0.0

        current_distance = obs['distance_to_source']
        current_concentration = self.plume_model.sample(self.robot_pos)

        # 1. 距离奖励（最重要的引导信号）
        if self.prev_distance is not None:
            distance_change = self.prev_distance - current_distance
            # 靠近源: +奖励, 远离源: -惩罚
            reward += distance_change * 15.0  # 放大系数

            # 额外：刷新最小距离记录时给大奖励
            if current_distance < self.min_distance:
                bonus = (self.min_distance - current_distance) * 20.0
                reward += bonus
                self.min_distance = current_distance

        # 2. 浓度梯度奖励
        concentration_change = current_concentration - self.prev_concentration
        if concentration_change > 0:
            # 浓度上升：正确方向
            reward += concentration_change * 30.0
        elif concentration_change < 0:
            # 浓度下降：错误方向
            reward += concentration_change * 10.0  # 惩罚较轻

        # 3. 羽流事件奖励。plume_stable 是状态而不是事件，不能每步重复奖励。
        if obs['plume_enter']:
            reward += 25.0  # 首次检测到plume

        if obs['plume_lost']:
            reward -= 10.0  # 失去plume惩罚

        # 4. 酒精概率提升奖励
        prob_change = obs['alcohol_prob'] - self.prev_alcohol_prob
        if prob_change > 0:
            reward += prob_change * 20.0

        # 5. 每步小代价，避免在高信号区域原地等待。
        reward -= 0.5

        # 位置输入必须对应可学习的边界代价。越过边界会终止回合；在
        # 边界预警带内提前惩罚，避免策略只在撞出场地后才获得反馈。
        boundary_margin = self._boundary_margin(self.robot_pos)
        if boundary_margin < 0:
            reward -= self.config.BOUNDARY_PENALTY
        elif boundary_margin < self.config.BOUNDARY_WARNING_MARGIN:
            reward -= ((self.config.BOUNDARY_WARNING_MARGIN - boundary_margin) /
                       self.config.BOUNDARY_WARNING_MARGIN *
                       self.config.BOUNDARY_WARNING_PENALTY)

        # 6. 弱传感器教师引导：只使用TGS当前观测，不读取源点真值。
        teacher_action = self._sensor_teacher_action(obs)
        if action == teacher_action:
            reward += self.config.TEACHER_GUIDANCE_WEIGHT
        else:
            reward -= self.config.TEACHER_MISMATCH_PENALTY

        return reward

    def _get_observation(self):
        """获取观测"""
        # 采样双传感器
        # 一个决策周期内模拟多个10 Hz传感器更新，策略仍以2 Hz读取末次值。
        sensor_steps = max(1, int(round(self.dt / self.sensor_dt)))
        tgs_left = tgs_right = 0.0
        for _ in range(sensor_steps):
            tgs_left, tgs_right = self.dual_sensor.sample(
                self.robot_pos, self.robot_yaw, self.plume_model, self.sensor_dt
            )

        tgs_mean = (tgs_left + tgs_right) / 2.0
        tgs_diff = tgs_left - tgs_right
        tgs_diff_norm = tgs_diff / (tgs_left + tgs_right + 1e-6)

        # BME688模拟
        concentration = self.plume_model.sample(self.robot_pos)
        alcohol_prob = self._concentration_to_prob(concentration)
        gas_resistance = 100000 - concentration * 1000

        temperature = 25.0 + self.rng.normal(0, 0.5)
        humidity = 50.0 + self.rng.normal(0, 2.0)

        # Plume事件检测
        plume_enter = (tgs_mean > 5.0 and self._prev_tgs_mean() < 5.0)
        plume_stable = (tgs_mean > 8.0)
        plume_lost = (tgs_mean < 3.0 and self._prev_tgs_mean() > 5.0)

        if plume_stable:
            self.consecutive_stable_steps += 1
        else:
            self.consecutive_stable_steps = 0

        distance_to_source = np.linalg.norm(self.robot_pos - self.source_pos)
        relative_heading = np.arctan2(
            np.sin(self.robot_yaw - self.initial_yaw),
            np.cos(self.robot_yaw - self.initial_yaw),
        )

        obs = {
            'alcohol_prob': alcohol_prob,
            'gas_resistance_norm': gas_resistance / 100000.0,
            'temperature': (temperature - 25.0) / 10.0,
            'humidity': (humidity - 50.0) / 20.0,
            'tgs_left_norm': tgs_left / 40.0,
            'tgs_right_norm': tgs_right / 40.0,
            'tgs_diff_norm': np.clip(tgs_diff_norm, -1, 1),
            'tgs_mean': tgs_mean,
            'tgs_slope': 0.0,
            'plume_enter': float(plume_enter),
            'plume_stable': float(plume_stable),
            'plume_lost': float(plume_lost),
            'linear_velocity': self.robot_vel[0] / 0.3,
            'angular_velocity': self.robot_vel[1] / 1.0,
            'gyro_z': self.robot_vel[1],
            'last_action': self.last_action / 6.0,
            'action_age': self.dt,
            'communication_quality': 1.0,
            'distance_to_source': distance_to_source,
            'steps': self.steps
        }
        if self.config.INCLUDE_POSITION:
            obs.update({
                'x_norm': (self.robot_pos[0] - self.field_width / 2.0) /
                          (self.field_width / 2.0),
                'y_norm': (self.robot_pos[1] - self.field_height / 2.0) /
                          (self.field_height / 2.0),
                'sin_heading': float(np.sin(relative_heading)),
                'cos_heading': float(np.cos(relative_heading)),
            })

        return obs

    def _check_done(self, obs, requested_action=None, executed_action=None):
        """检查终止条件"""
        info = {
            'success': False,
            'collision': False,
            'timeout': False,
            'requested_action': int(self.last_action if requested_action is None else requested_action),
            'executed_action': int(self.last_action if executed_action is None else executed_action),
            'safety_override': bool(requested_action is not None and
                                    requested_action != executed_action),
            'distance_to_source': float(obs['distance_to_source']),
        }

        # 成功条件
        if (obs['distance_to_source'] < self.success_radius and
            obs['alcohol_prob'] > self.success_threshold and
            self.consecutive_stable_steps >= self.success_window):
            info['success'] = True
            return True, info

        # 碰撞
        if not self._in_bounds(self.robot_pos):
            info['collision'] = True
            return True, info

        # 超时
        if self.steps >= self.max_steps:
            info['timeout'] = True
            return True, info

        return False, info

    def _get_stacked_state(self):
        """堆叠状态"""
        return np.array(list(self.state_history))

    def _obs_to_vec(self, obs):
        """观测转向量"""
        values = [obs['alcohol_prob']]
        if self.config.INCLUDE_GAS_RESISTANCE:
            values.append(obs['gas_resistance_norm'])
        values.extend([
            obs['temperature'], obs['humidity'],
            obs['tgs_left_norm'], obs['tgs_right_norm'], obs['tgs_diff_norm'],
            obs['tgs_mean'] / 40.0, obs['tgs_slope'],
            obs['plume_enter'], obs['plume_stable'], obs['plume_lost'],
            obs['linear_velocity'], obs['angular_velocity'],
            obs['gyro_z'],
            obs['last_action'], obs['action_age'], obs['communication_quality']
        ])
        if self.config.INCLUDE_POSITION:
            values.extend([
                obs['x_norm'], obs['y_norm'],
                obs['sin_heading'], obs['cos_heading'],
            ])
        return np.array(values, dtype=np.float32)

    def _concentration_to_prob(self, concentration):
        """浓度转概率"""
        return 1.0 / (1.0 + np.exp(-5.0 * (concentration - 10.0)))

    def _prev_tgs_mean(self):
        """获取前一步TGS均值"""
        return self.prev_tgs_mean

    def _sensor_teacher_action(self, obs):
        """传感器可见的方向启发式，仅用于训练奖励，不进入策略输入。"""
        tgs_diff = (obs['tgs_diff_norm']
                    * (obs['tgs_left_norm'] + obs['tgs_right_norm']) * 40.0)
        tgs_mean = obs['tgs_mean']
        if abs(tgs_diff) > 0.015:
            return 3 if tgs_diff > 0 else 4
        if tgs_mean < 4.8:
            last_action = int(round(obs['last_action'] * 6.0))
            return 3 if last_action != 3 else 4
        return 2

    def _in_bounds(self, pos):
        """检查是否在边界内"""
        return (self.robot_radius <= pos[0] <= self.field_width - self.robot_radius and
                self.robot_radius <= pos[1] <= self.field_height - self.robot_radius)

    def _boundary_margin(self, pos):
        """Return clearance from the robot-radius boundary; negative is outside."""
        return min(
            pos[0] - self.robot_radius,
            self.field_width - self.robot_radius - pos[0],
            pos[1] - self.robot_radius,
            self.field_height - self.robot_radius - pos[1],
        )

    def _boundary_safe_action(self, action):
        """Override only a motion command whose next pose would leave the map."""
        v, omega = self.VELOCITY_MAP[self.ACTIONS[action]]
        if v <= 0:
            return action
        next_yaw = self.robot_yaw + omega * self.dt
        next_pos = self.robot_pos + np.array([
            v * np.cos(next_yaw) * self.dt,
            v * np.sin(next_yaw) * self.dt,
        ])
        if self._boundary_margin(next_pos) >= self.config.BOUNDARY_SAFETY_MARGIN:
            return action
        center = np.array([self.field_width / 2.0, self.field_height / 2.0])
        target = np.arctan2(*(center - self.robot_pos)[::-1])
        error = np.arctan2(
            np.sin(target - self.robot_yaw),
            np.cos(target - self.robot_yaw),
        )
        if error > 0.35:
            return self.ACTIONS.index('TURN_LEFT')
        if error < -0.35:
            return self.ACTIONS.index('TURN_RIGHT')
        return self.ACTIONS.index('FORWARD_SLOW')
