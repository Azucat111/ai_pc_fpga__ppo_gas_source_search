"""
真实传感器配置
基于 TGS2602 和 BME688 实测性能
"""


class RealisticEnvironmentConfig:
    """增强的真实环境配置"""

    # 场地设置
    FIELD_SIZE = (3.0, 3.0)  # 3m × 3m
    GRID_RESOLUTION = 0.1

    # 机器人参数
    ROBOT_RADIUS = 0.15
    MAX_LINEAR_VEL = 0.3
    MAX_ANGULAR_VEL = 1.0

    # 动作空间（新增后退动作）
    VELOCITY_MAP = {
        'STOP': (0.0, 0.0),
        'FORWARD_SLOW': (0.1, 0.0),
        'FORWARD': (0.2, 0.0),
        'BACKWARD_SLOW': (-0.1, 0.0),  # 新增
        'TURN_LEFT': (0.0, 0.8),
        'TURN_RIGHT': (0.0, -0.8),
        'ARC_LEFT': (0.15, 0.5),
        'ARC_RIGHT': (0.15, -0.5),
    }

    # 时间设置
    DT = 0.1  # 仿真时间步长(秒)
    STATE_UPDATE_FREQ = 10  # 10Hz
    DECISION_FREQ = 2  # 2Hz

    # === TGS2602 真实参数（基于实测数据）===
    TGS2602_TAU_RISE_MIN = 2.0      # 上升时间常数 2-3s
    TGS2602_TAU_RISE_MAX = 3.0
    TGS2602_TAU_FALL_MIN = 60.0     # 下降时间常数 60-90s (关键!)
    TGS2602_TAU_FALL_MAX = 90.0
    TGS2602_BASELINE = 0.075        # 基线电压 0.066-0.085V
    TGS2602_BASELINE_DRIFT = 0.00001  # 基线漂移 ±0.005V std
    TGS2602_NOISE_STD = 0.004       # 噪声 ±0.005V std
    TGS2602_MAX_RESPONSE = 4.85     # 最大响应 +485%
    TGS2602_DISTANCE_RANGE = (0.2, 2.0)  # 有效检测距离 0.2-2m
    TGS2602_DIRECTION_RESOLUTION = 15.0  # 方向分辨率 ±15°

    # === BME688 真实参数 ===
    BME688_SCAN_PERIOD = 2.5        # 扫描周期 2.5s/样本
    BME688_CALIBRATION_SAMPLES = 20  # 启动标定 20样本 = 50秒
    BME688_RECOVERY_TIME = 20.0     # 恢复时间 10-30s
    BME688_ACCURACY = 0.98          # 分类准确率 100% → 98%（考虑噪声）

    # === 增强的羽流模型参数 ===
    PLUME_SOURCE_STRENGTH = 30.0    # 源强度
    PLUME_WIND_SPEED = 0.2          # 平均风速 0.2 m/s
    PLUME_WIND_DIRECTION = 0.0      # 主导风向 (rad)
    PLUME_WIND_VARIABILITY = 0.3    # 风向变异性(rad)
    PLUME_TURBULENCE = 0.2          # 湍流强度 20%
    PLUME_INTERMITTENCY = 0.7       # 间歇性 70%（vs 原80%）
    PLUME_FILAMENT_COUNT = 5        # 丝状结构数量
    PLUME_DIFFUSION = 0.1           # 扩散系数

    # === 初始条件约束（增强难度）===
    INIT_DISTANCE_MIN = 1.5         # 最小初始距离 1.0→1.5m
    INIT_DISTANCE_MAX = 3.5         # 最大初始距离（新增上限）
    INIT_HEADING_BIAS_ENABLED = False  # 禁用朝向偏向
    SOURCE_POSITION_MARGIN = 0.5    # 源点距边界距离

    # 成功判定（保持方案E的放宽条件）
    SUCCESS_RADIUS = 0.8
    SUCCESS_THRESHOLD = 0.5
    SUCCESS_WINDOW = 1

    # Episode设置
    MAX_STEPS = 150  # 100→150步（考虑传感器延迟）

    # 羽流模型兼容参数（兼容原有环境）
    PLUME_DECAY = 0.004
    PLUME_WIDTH = 1.0
    PLUME_SCALING = 30.0
    WIND_TURBULENCE = True
    TURBULENCE_SCALE = 0.15

    # 传感器参数（兼容原有环境）
    TGS2602_TAU = 2.5  # 使用真实上升时间
    TGS2602_RECOVERY_TAU = 75.0  # 使用真实恢复时间
    SENSOR_NOISE_STD = 0.004


class RealisticRewardConfig:
    """
    真实传感器下的奖励配置
    需要考虑：
    1. TGS2602 恢复期很长（60-90s）
    2. BME688 延迟大（2.5s扫描周期）
    3. 信号间歇性和湍流
    """

    # Level 1: 生存与探索
    STEP_COST = 0.08  # 降低（0.1→0.08），因为增加了步数上限
    MOVEMENT_BONUS = 0.05

    # Level 2: 信号检测（延迟补偿）
    PLUME_DETECTION_REWARD = 8.0  # 提高（5.0→8.0），因为检测更难
    EVIDENCE_GAIN_WEIGHT = 4.0    # 提高（3.0→4.0）

    # Level 3: 方向寻优
    DIRECTION_GUIDANCE_REWARD = 3.0  # 提高（2.0→3.0）

    # Level 4: 目标接近
    APPROACH_BASE_REWARD = 20.0   # 提高（15.0→20.0）
    APPROACH_DISTANCE_SCALING = True
    APPROACH_MIN_DISTANCE = 0.3

    # Level 5: 成功完成
    SUCCESS_REWARD = 250.0  # 提高（200.0→250.0）

    # 终止惩罚
    COLLISION_PENALTY = 15.0  # 提高（10.0→15.0）
    TIMEOUT_PENALTY = 5.0     # 提高（3.0→5.0）
    FAULT_PENALTY = 15.0

    # 行为塑形
    OSCILLATION_PENALTY = 0.15  # 提高（0.1→0.15）

    # 新增：传感器恢复期惩罚
    RECOVERY_PERIOD_PENALTY = 0.5  # 在恢复期移动的惩罚


class DifficultyLevel:
    """
    难度等级配置
    允许渐进式训练
    """

    @staticmethod
    def easy():
        """简单难度（适合初期训练）"""
        config = RealisticEnvironmentConfig()
        config.INIT_DISTANCE_MIN = 1.0
        config.INIT_DISTANCE_MAX = 2.0
        config.INIT_HEADING_BIAS_ENABLED = True  # 朝向偏向气源
        config.PLUME_TURBULENCE = 0.1
        config.PLUME_INTERMITTENCY = 0.9
        config.TGS2602_TAU_FALL_MIN = 30.0  # 减半恢复时间
        config.TGS2602_TAU_FALL_MAX = 45.0
        config.MAX_STEPS = 120
        return config

    @staticmethod
    def medium():
        """中等难度"""
        config = RealisticEnvironmentConfig()
        config.INIT_DISTANCE_MIN = 1.2
        config.INIT_DISTANCE_MAX = 2.5
        config.PLUME_TURBULENCE = 0.15
        config.PLUME_INTERMITTENCY = 0.8
        config.MAX_STEPS = 150
        return config

    @staticmethod
    def hard():
        """困难难度（接近真实条件）"""
        return RealisticEnvironmentConfig()

    @staticmethod
    def expert():
        """专家难度（超出真实条件）"""
        config = RealisticEnvironmentConfig()
        config.INIT_DISTANCE_MIN = 2.0
        config.INIT_DISTANCE_MAX = 4.0
        config.PLUME_TURBULENCE = 0.3
        config.PLUME_INTERMITTENCY = 0.6
        config.TGS2602_NOISE_STD = 0.008  # 加倍噪声
        config.MAX_STEPS = 200
        return config


class DataPreprocessingConfig:
    """
    传感器数据预处理配置
    模拟真实系统的滤波流程
    """

    # === TGS2602 多层滤波（参考实测系统）===
    HARDWARE_AVERAGE_SAMPLES = 10  # 硬件平均
    KALMAN_Q = 1e-5               # 卡尔曼过程噪声
    KALMAN_R = 5e-3               # 卡尔曼测量噪声
    HYSTERESIS_LOW = 0.02         # 迟滞检测器低阈值(V)
    HYSTERESIS_HIGH = 0.05        # 迟滞检测器高阈值(V)
    DEAD_ZONE = 0.01              # 差分死区(V)
    CONFIRMATION_WINDOW = 5       # 时间窗口确认(样本数)
    CONFIRMATION_RATIO = 0.6      # 确认比例

    # === 自适应基线跟踪 ===
    BASELINE_ALPHA_DECAY = 0.01   # 下降时更新率
    BASELINE_ALPHA_RISE = 0.0001  # 上升时更新率（慢很多）

    # === 时间导数计算 ===
    DERIVATIVE_WINDOW = 10        # 导数计算窗口(样本数)
    DERIVATIVE_THRESHOLD_RISE = 0.02   # 接近判定(V/s)
    DERIVATIVE_THRESHOLD_FALL = -0.01  # 远离判定(V/s)
    DERIVATIVE_THRESHOLD_STABLE = 0.005  # 稳定判定(V/s)

    # === BME688 特征工程 ===
    BME688_BASELINE_NORMALIZE = True  # 基线归一化
    BME688_STANDARD_SCALE = True      # 标准化
    BME688_FEATURE_DIM = 10           # 10维电阻特征


# 性能基准（用于评估）
class PerformanceBenchmark:
    """
    真实传感器下的预期性能基准
    基于 TGS2602 技术文档
    """

    # TGS2602 系统（纯差分导航）
    TGS2602_EXPECTED_SUCCESS_RATE = 0.60  # 60-75%
    TGS2602_AVG_LOCALIZATION_TIME = 135   # 90-180s
    TGS2602_POSITION_ERROR = 0.5          # ±0.5m

    # DQN（当前实现）
    DQN_CURRENT_SUCCESS_RATE = 0.15       # 15%
    DQN_AVG_STEPS = 85
    DQN_SUCCESS_AVG_STEPS = 68

    # 融合系统（目标）
    FUSION_TARGET_SUCCESS_RATE = 0.90     # 90-98%
    FUSION_TARGET_TIME = 60               # <60s


# 导出便捷别名
RealisticConfig = RealisticEnvironmentConfig
RealisticReward = RealisticRewardConfig
Difficulty = DifficultyLevel
Preprocessing = DataPreprocessingConfig
Benchmark = PerformanceBenchmark
