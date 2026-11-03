"""
羽流扩散模型 (Plume Model)
参考: DQN_OSL-main/Plume.py
"""

import numpy as np


class GaussianPlumeModel:
    """
    简化无风高斯扩散模型 (各向同性)

    模型公式:
        C(r) = scaling * exp(-0.5 * (r / width(r))^2) * decay_term
    """

    def __init__(self, source_pos, diffusion=0.1, decay=0.004,
                 base_width=1.0, scaling=30.0):
        """
        Args:
            source_pos: (x, y) 源点位置
            diffusion: 扩散系数 (控制横向扩散速度)
            decay: 衰减系数 (控制纵向衰减速度)
            base_width: 基础羽流宽度
            scaling: 幅值缩放 (源强度)
        """
        self.source_x = source_pos[0]
        self.source_y = source_pos[1]
        self.diffusion = diffusion
        self.decay = decay
        self.base_width = base_width
        self.scaling = scaling

    def sample(self, position):
        """
        在指定位置采样浓度

        Args:
            position: (x, y) 采样位置

        Returns:
            concentration: float (浓度值, ≥0)
        """
        dx = position[0] - self.source_x
        dy = position[1] - self.source_y
        radius = np.hypot(dx, dy)
        width = self.base_width + radius * self.diffusion
        radial_term = np.exp(-0.5 * (radius / max(width, 1e-6)) ** 2)
        decay_term = max(0.0, 1.0 - self.decay * radius ** 2)
        concentration = self.scaling * radial_term * decay_term

        return max(0.0, concentration)  # 确保非负

    def sample_grid(self, x_range, y_range, resolution=0.1):
        """
        在网格上采样 (用于可视化)

        Args:
            x_range: (x_min, x_max)
            y_range: (y_min, y_max)
            resolution: 网格分辨率

        Returns:
            X, Y, C: 网格坐标和浓度
        """
        x = np.arange(x_range[0], x_range[1], resolution)
        y = np.arange(y_range[0], y_range[1], resolution)
        X, Y = np.meshgrid(x, y)

        C = np.zeros_like(X)
        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                C[i, j] = self.sample((X[i, j], Y[i, j]))

        return X, Y, C


class SensorDynamics:
    """
    传感器动态模型 (一阶滞后)
    """

    def __init__(self, tau_response=5.0, tau_recovery=30.0, noise_std=0.05):
        """
        Args:
            tau_response: 响应时间常数 (秒)
            tau_recovery: 恢复时间常数 (秒)
            noise_std: 测量噪声标准差
        """
        self.tau_response = tau_response
        self.tau_recovery = tau_recovery
        self.noise_std = noise_std

        # 传感器状态
        self.output = 0.0

    def update(self, true_concentration, dt=0.1):
        """
        更新传感器输出 (一阶低通滤波)

        Args:
            true_concentration: 真实浓度
            dt: 时间步长 (秒)

        Returns:
            sensor_output: 传感器输出值
        """
        # 选择时间常数 (上升快, 下降慢)
        if true_concentration > self.output:
            tau = self.tau_response
        else:
            tau = self.tau_recovery

        # 一阶滞后: dS/dt = (C - S) / τ
        self.output += dt * (true_concentration - self.output) / tau

        # 添加测量噪声
        noise = np.random.normal(0, self.noise_std)
        measured = self.output + noise

        return max(0.0, measured)  # 传感器输出非负

    def reset(self):
        """重置传感器状态"""
        self.output = 0.0


class DualTGS2602Sensor:
    """
    双TGS2602传感器 (左右通道)
    """

    def __init__(self, robot_offset=0.15, tau_response=5.0, tau_recovery=30.0,
                 gain_left=1.0, gain_right=1.0, noise_std=0.05):
        """
        Args:
            robot_offset: 传感器相对机器人中心的横向偏移 (米)
            tau_response/tau_recovery: 时间常数
            gain_left/gain_right: 左右通道增益 (模拟硬件差异)
            noise_std: 噪声标准差
        """
        self.offset = robot_offset
        self.gain_left = gain_left
        self.gain_right = gain_right

        # 左右传感器动态
        self.sensor_left = SensorDynamics(tau_response, tau_recovery, noise_std)
        self.sensor_right = SensorDynamics(tau_response, tau_recovery, noise_std)

    def sample(self, robot_pos, robot_yaw, plume_model, dt=0.1):
        """
        采样左右传感器

        Args:
            robot_pos: (x, y) 机器人位置
            robot_yaw: 机器人航向角 (弧度)
            plume_model: 羽流模型实例
            dt: 时间步长

        Returns:
            left_output, right_output: 左右传感器输出
        """
        # 传感器在机器人坐标系中的位置 (假设在前方两侧)
        # 左传感器: 相对机器人中心 (向前0.1m, 向左0.15m)
        # 右传感器: 相对机器人中心 (向前0.1m, 向右0.15m)
        forward_offset = -0.1  # 在机器人前方

        # 旋转到世界坐标系
        cos_yaw = np.cos(robot_yaw)
        sin_yaw = np.sin(robot_yaw)

        left_local = np.array([forward_offset, self.offset])
        right_local = np.array([forward_offset, -self.offset])

        left_global = robot_pos + np.array([
            left_local[0] * cos_yaw - left_local[1] * sin_yaw,
            left_local[0] * sin_yaw + left_local[1] * cos_yaw
        ])

        right_global = robot_pos + np.array([
            right_local[0] * cos_yaw - right_local[1] * sin_yaw,
            right_local[0] * sin_yaw + right_local[1] * cos_yaw
        ])

        # 采样真实浓度
        C_left_true = plume_model.sample(left_global) * self.gain_left
        C_right_true = plume_model.sample(right_global) * self.gain_right

        # 通过传感器动态
        left_output = self.sensor_left.update(C_left_true, dt)
        right_output = self.sensor_right.update(C_right_true, dt)

        return left_output, right_output

    def reset(self):
        """重置传感器状态"""
        self.sensor_left.reset()
        self.sensor_right.reset()


if __name__ == "__main__":
    # 测试羽流模型
    print("=== 测试高斯羽流模型 ===")
    plume = GaussianPlumeModel(source_pos=(2.0, 2.0), scaling=30.0)

    test_positions = [
        (2.0, 2.0),  # 源点
        (2.5, 2.0),  # 源点前方0.5m
        (1.5, 2.0),  # 源点后方0.5m
        (2.5, 2.3),  # 前方偏左
        (2.5, 1.7),  # 前方偏右
    ]

    for pos in test_positions:
        C = plume.sample(pos)
        print(f"位置 {pos}: 浓度 = {C:.2f}")

    # 测试传感器动态
    print("\n=== 测试传感器滞后 ===")
    sensor = SensorDynamics(tau_response=5.0, tau_recovery=30.0, noise_std=0.0)

    # 阶跃响应测试
    print("阶跃输入: 0 → 10")
    for t in range(50):
        output = sensor.update(10.0 if t < 25 else 0.0, dt=0.1)
        if t % 5 == 0:
            print(f"  t={t*0.1:.1f}s: 输出={output:.2f}")

    # 测试双传感器
    print("\n=== 测试双TGS2602传感器 ===")
    dual_sensor = DualTGS2602Sensor(robot_offset=0.15, tau_response=5.0)
    dual_sensor.reset()

    robot_pos = np.array([1.5, 2.0])
    robot_yaw = 0.0  # 朝向正东

    print("机器人位置:", robot_pos, "航向角:", robot_yaw)
    for step in range(20):
        left, right = dual_sensor.sample(robot_pos, robot_yaw, plume, dt=0.1)
        if step % 5 == 0:
            print(f"  Step {step}: Left={left:.2f}, Right={right:.2f}, Diff={left-right:.2f}")
