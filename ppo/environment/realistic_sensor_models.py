"""
增强的真实传感器模型
基于 TGS2602 和 BME688 实测性能
参考: TGS2602_Technical_Spec.md, BME688系统功能与性能说明.md
"""

import numpy as np


class RealisticTGS2602Sensor:
    """
    真实 TGS2602 传感器动态模型

    关键特性:
    1. 非对称响应：上升快(2-3s)，下降慢(60-90s)
    2. 基线漂移：长期运行漂移 ±0.005V
    3. 非线性响应曲线
    4. 温度依赖性
    5. 传感器老化
    """

    def __init__(self,
                 tau_rise=2.5,           # 上升时间常数 2-3s
                 tau_fall=75.0,          # 下降时间常数 60-90s
                 baseline_voltage=0.075, # 基线电压 (clean air)
                 baseline_drift_rate=0.00001,  # 基线漂移速率 V/step
                 noise_std=0.004,        # 噪声标准差 ~0.005V
                 max_response_ratio=4.85,  # 最大响应倍数 +485%
                 distance_decay=0.5,     # 距离衰减指数
                 saturation_conc=50.0):  # 饱和浓度
        """
        Args:
            tau_rise: 上升时间常数(秒)
            tau_fall: 下降时间常数(秒)，应远大于tau_rise
            baseline_voltage: 清洁空气下的基线电压(V)
            baseline_drift_rate: 基线漂移速率(V/step)
            noise_std: 测量噪声标准差(V)
            max_response_ratio: 最大响应倍数
            distance_decay: 距离衰减指数
            saturation_conc: 饱和浓度
        """
        self.tau_rise = tau_rise
        self.tau_fall = tau_fall
        self.baseline_voltage = baseline_voltage
        self.baseline_drift_rate = baseline_drift_rate
        self.noise_std = noise_std
        self.max_response_ratio = max_response_ratio
        self.distance_decay = distance_decay
        self.saturation_conc = saturation_conc

        # 传感器状态
        self.current_voltage = baseline_voltage
        self.current_baseline = baseline_voltage
        self.total_exposure_time = 0.0  # 累计暴露时间（模拟老化）

        # 历史记录（用于时间导数）
        self.voltage_history = []

    def _nonlinear_response(self, concentration):
        """
        非线性响应曲线 (类似对数响应)

        Args:
            concentration: 真实浓度

        Returns:
            response_ratio: 相对基线的响应倍数
        """
        if concentration <= 0:
            return 1.0

        # 饱和响应曲线: R = 1 + (R_max - 1) * (1 - exp(-k*C))
        k = 3.0 / self.saturation_conc  # 响应速率常数
        response = 1.0 + (self.max_response_ratio - 1.0) * (1.0 - np.exp(-k * concentration))

        return response

    def _baseline_drift(self, dt):
        """
        基线漂移（随机游走 + 长期漂移）

        Args:
            dt: 时间步长(秒)
        """
        # 随机游走
        random_drift = np.random.normal(0, self.baseline_drift_rate * np.sqrt(dt))

        # 长期漂移（老化效应，单向）
        aging_drift = self.baseline_drift_rate * dt * 0.1

        self.current_baseline += random_drift + aging_drift

        # 限制漂移范围
        self.current_baseline = np.clip(
            self.current_baseline,
            self.baseline_voltage * 0.95,
            self.baseline_voltage * 1.05
        )

    def update(self, true_concentration, dt=0.1):
        """
        更新传感器状态（非对称一阶滞后）

        Args:
            true_concentration: 真实气体浓度
            dt: 时间步长(秒)

        Returns:
            measured_voltage: 测量电压(V)
        """
        # 目标电压
        response_ratio = self._nonlinear_response(true_concentration)
        target_voltage = self.current_baseline * response_ratio

        # 非对称时间常数
        if target_voltage > self.current_voltage:
            tau = self.tau_rise  # 上升快
        else:
            tau = self.tau_fall  # 下降慢 (关键!)

        # 一阶滞后: dV/dt = (V_target - V) / τ
        dV = (target_voltage - self.current_voltage) * dt / tau
        self.current_voltage += dV

        # 基线漂移
        self._baseline_drift(dt)

        # 测量噪声（高斯白噪声）
        noise = np.random.normal(0, self.noise_std)
        measured = self.current_voltage + noise

        # 记录历史
        self.voltage_history.append(measured)
        if len(self.voltage_history) > 50:  # 保留5秒历史(假设10Hz)
            self.voltage_history.pop(0)

        # 累计暴露时间（用于老化模拟）
        if true_concentration > 1.0:
            self.total_exposure_time += dt

        return max(0.0, measured)

    def get_derivative(self, window=10):
        """
        计算时间导数 dV/dt

        Args:
            window: 窗口大小(样本数)

        Returns:
            derivative: V/s
        """
        if len(self.voltage_history) < window:
            return 0.0

        recent = self.voltage_history[-window:]
        # 线性回归斜率
        x = np.arange(len(recent))
        slope = np.polyfit(x, recent, 1)[0]

        # 转换为 V/s (假设10Hz采样)
        return slope * 10.0

    def reset(self):
        """重置传感器状态"""
        self.current_voltage = self.baseline_voltage
        self.current_baseline = self.baseline_voltage
        self.voltage_history = []
        self.total_exposure_time = 0.0


class RealisticBME688Sensor:
    """
    真实 BME688 气体分类传感器模型

    关键特性:
    1. 扫描周期：2.5秒/样本（10点加热器扫描）
    2. 启动标定：50秒（20样本）
    3. 10维特征向量
    4. 恢复时间：10-30秒
    5. 分类准确率：100% (理想条件)
    """

    def __init__(self,
                 scan_period=2.5,      # 扫描周期(秒)
                 calibration_samples=20,  # 标定样本数
                 recovery_time=20.0,   # 恢复时间(秒)
                 accuracy=0.98):       # 分类准确率
        """
        Args:
            scan_period: 扫描周期(秒)
            calibration_samples: 启动标定所需样本数
            recovery_time: 恢复时间(秒)
            accuracy: 分类准确率(0-1)
        """
        self.scan_period = scan_period
        self.calibration_samples = calibration_samples
        self.recovery_time = recovery_time
        self.accuracy = accuracy

        # 传感器状态
        self.is_calibrated = False
        self.sample_count = 0
        self.last_scan_time = 0.0
        self.residual_response = 0.0  # 残留响应（恢复期）

    def update(self, true_concentration, time_elapsed, dt=0.1):
        """
        更新传感器状态

        Args:
            true_concentration: 真实气体浓度
            time_elapsed: 从episode开始的总时间(秒)
            dt: 时间步长(秒)

        Returns:
            alcohol_probability: 酒精概率(0-1)
            is_valid: 数据是否有效(是否到扫描时刻)
        """
        # 检查是否到达扫描时刻
        if time_elapsed - self.last_scan_time < self.scan_period:
            return 0.0, False

        # 执行扫描
        self.last_scan_time = time_elapsed
        self.sample_count += 1

        # 标定期
        if self.sample_count < self.calibration_samples:
            self.is_calibrated = False
            return 0.0, False

        self.is_calibrated = True

        # 恢复期处理（残留响应指数衰减）
        tau_recovery = self.recovery_time / 3.0
        self.residual_response *= np.exp(-self.scan_period / tau_recovery)

        # 当前响应 + 残留响应
        current_response = true_concentration + self.residual_response

        # 更新残留
        if true_concentration > 0:
            self.residual_response = current_response

        # Sigmoid 响应曲线
        threshold = 5.0  # 响应阈值
        prob_true = 1.0 / (1.0 + np.exp(-2 * (current_response - threshold)))

        # 模拟分类误差
        if np.random.rand() < (1.0 - self.accuracy):
            prob_true = 1.0 - prob_true  # 翻转预测

        return prob_true, True

    def reset(self):
        """重置传感器状态"""
        self.is_calibrated = False
        self.sample_count = 0
        self.last_scan_time = 0.0
        self.residual_response = 0.0


class RealisticPlumeModel:
    """
    增强的真实羽流模型

    新增特性:
    1. 风向扰动（随机方向+速度）
    2. 湍流涡旋（Perlin噪声）
    3. 羽流间歇性（丝状结构）
    4. 浓度脉动（时间变化）
    5. 源点强度波动
    """

    def __init__(self,
                 source_pos,
                 source_strength=30.0,
                 wind_speed=0.2,        # 平均风速(m/s)
                 wind_direction=0.0,    # 风向(rad)
                 wind_variability=0.3,  # 风向变异性
                 turbulence_intensity=0.2,  # 湍流强度
                 intermittency=0.7,     # 羽流间歇性(0-1)
                 filament_count=5,      # 丝状结构数量
                 diffusion_coef=0.1):
        """
        Args:
            source_pos: 源点位置 (x, y)
            source_strength: 源强度
            wind_speed: 平均风速(m/s)
            wind_direction: 主导风向(rad)
            wind_variability: 风向标准差(rad)
            turbulence_intensity: 湍流强度(0-1)
            intermittency: 羽流存在概率(0-1)
            filament_count: 丝状结构数量
            diffusion_coef: 扩散系数
        """
        self.source_pos = np.array(source_pos)
        self.base_strength = source_strength
        self.current_strength = source_strength
        self.wind_speed = wind_speed
        self.wind_direction = wind_direction
        self.wind_variability = wind_variability
        self.turbulence_intensity = turbulence_intensity
        self.intermittency = intermittency
        self.filament_count = filament_count
        self.diffusion_coef = diffusion_coef

        # 时变参数
        self.time = 0.0
        self.current_wind_dir = wind_direction
        self.filament_positions = []  # 丝状结构位置

    def _update_wind(self, dt):
        """更新风向（随机游走）"""
        # 风向扰动
        wind_change = np.random.normal(0, self.wind_variability * np.sqrt(dt))
        self.current_wind_dir += wind_change

        # 源强度波动（±20%）
        strength_noise = np.random.normal(1.0, 0.1)
        self.current_strength = self.base_strength * strength_noise

    def _turbulence_effect(self, pos):
        """
        湍流扰动（简化Perlin噪声）

        Args:
            pos: 位置 (x, y)

        Returns:
            perturbation: 浓度扰动系数(0-2)
        """
        # 使用时间和空间的周期函数模拟湍流
        x, y = pos
        t = self.time

        turb_x = np.sin(x * 2.0 + t * 0.5) * np.cos(y * 3.0)
        turb_y = np.cos(x * 3.0) * np.sin(y * 2.0 + t * 0.7)
        turb_t = np.sin(t * 1.3)

        turbulence = (turb_x + turb_y + turb_t) / 3.0

        # 映射到 [1-intensity, 1+intensity]
        return 1.0 + turbulence * self.turbulence_intensity

    def _filament_structure(self, pos):
        """
        丝状羽流结构（间歇性）

        Args:
            pos: 位置 (x, y)

        Returns:
            filament_factor: 丝状增强系数(0-2)
        """
        # 随机生成丝状中心
        if len(self.filament_positions) == 0 or np.random.rand() < 0.1:
            # 沿风向生成丝状结构
            self.filament_positions = []
            for _ in range(self.filament_count):
                dist = np.random.exponential(1.0)
                angle = self.current_wind_dir + np.random.normal(0, 0.3)
                fil_pos = self.source_pos + dist * np.array([np.cos(angle), np.sin(angle)])
                self.filament_positions.append(fil_pos)

        # 计算到最近丝状中心的距离
        if len(self.filament_positions) == 0:
            return 1.0

        distances = [np.linalg.norm(pos - fil) for fil in self.filament_positions]
        min_dist = min(distances)

        # 高斯增强
        filament_width = 0.3  # 丝状宽度(m)
        enhancement = 2.0 * np.exp(-(min_dist / filament_width) ** 2)

        return max(0.0, enhancement)

    def sample(self, position, dt=0.1):
        """
        采样指定位置的浓度

        Args:
            position: 采样位置 (x, y)
            dt: 时间步长(秒)

        Returns:
            concentration: 浓度值
        """
        self.time += dt
        self._update_wind(dt)

        # 相对位置
        dx_vec = position - self.source_pos
        distance = np.linalg.norm(dx_vec)

        if distance < 0.1:
            return self.current_strength

        # 沿风向的距离
        wind_vec = np.array([np.cos(self.current_wind_dir),
                            np.sin(self.current_wind_dir)])
        downwind_dist = np.dot(dx_vec, wind_vec)
        crosswind_dist = np.linalg.norm(dx_vec - downwind_dist * wind_vec)

        # 羽流只在下风向
        if downwind_dist < 0:
            return 0.0

        # 基础高斯羽流
        sigma_y = self.diffusion_coef * (downwind_dist + 0.5)
        gaussian_factor = np.exp(-(crosswind_dist / sigma_y) ** 2)

        # 距离衰减
        decay = 1.0 / (1.0 + distance ** 1.5)

        # 基础浓度
        base_conc = self.current_strength * gaussian_factor * decay

        # 湍流扰动
        turb_factor = self._turbulence_effect(position)

        # 丝状结构
        filament_factor = self._filament_structure(position)

        # 间歇性（随机缺失）
        if np.random.rand() > self.intermittency:
            return 0.0

        concentration = base_conc * turb_factor * filament_factor

        return max(0.0, concentration)

    def reset(self):
        """重置羽流状态"""
        self.time = 0.0
        self.current_wind_dir = self.wind_direction
        self.current_strength = self.base_strength
        self.filament_positions = []
