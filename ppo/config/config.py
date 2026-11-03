"""
DQN训练配置文件
根据 单车气味源定位强化学习方案.md 和 DQN_OSL参考代码设计
"""

class DQNConfig:
    """DQN训练超参数"""

    # 网络参数
    STATE_SHAPE = (8, 18)  # PPO v3观测；不含障碍物输入
    NUM_ACTIONS = 7

    # 训练参数
    BATCH_SIZE = 32
    LEARNING_RATE = 1e-4
    GAMMA = 0.9  # 折扣因子

    # 探索策略 (方案E: 分阶段衰减)
    EPSILON_START = 1.0
    EPSILON_PHASE1_END = 0.3    # 第一阶段目标
    EPSILON_PHASE1_EPISODES = 1500  # 前1500 episodes快速探索
    EPSILON_PHASE2_END = 0.05   # 第二阶段目标
    EPSILON_PHASE2_EPISODES = 1500  # 后1500 episodes缓慢收敛
    EPSILON_MIN = 0.05

    # 经验回放
    MEMORY_SIZE = 60000
    MIN_MEMORY_SIZE = 1000  # 开始训练前的最小经验数

    # 目标网络更新
    TARGET_UPDATE_FREQ = 1000  # 每N步同步目标网络

    # 训练设置
    NUM_EPISODES = 300000
    MAX_STEPS_PER_EPISODE = 120  # 2 Hz × 60 s

    # 日志和保存
    LOG_INTERVAL = 20  # 每N局记录一次
    SAVE_INTERVAL = 1000  # 每N局保存一次模型

    # 设备
    DEVICE = 'cuda'  # 'cuda' or 'cpu' - AMD Radeon 8060S (ROCm 10.0.0)


class RewardConfig:
    """奖励函数参数 (方案E: 分层奖励)"""

    # Level 1: 生存与探索
    STEP_COST = 0.1  # 降低step cost鼓励长期探索
    MOVEMENT_BONUS = 0.05  # 鼓励移动(非STOP)

    # Level 2: 信号检测 (感知层)
    PLUME_DETECTION_REWARD = 5.0  # 检测到plume (alcohol_prob > 0.3)
    EVIDENCE_GAIN_WEIGHT = 3.0  # 置信度提升奖励权重

    # Level 3: 方向寻优 (导航层)
    DIRECTION_GUIDANCE_REWARD = 2.0  # 朝浓度高方向前进

    # Level 4: 目标接近 (任务层)
    APPROACH_BASE_REWARD = 15.0  # 基础接近奖励
    APPROACH_DISTANCE_SCALING = True  # 越近奖励越高
    APPROACH_MIN_DISTANCE = 0.3  # 距离因子下限

    # Level 5: 成功完成 (终极目标)
    SUCCESS_REWARD = 200.0  # 巨大成功奖励

    # 终止惩罚
    COLLISION_PENALTY = 10.0
    TIMEOUT_PENALTY = 3.0
    FAULT_PENALTY = 10.0

    # 行为塑形
    OSCILLATION_PENALTY = 0.1


class EnvironmentConfig:
    """仿真环境参数"""

    # 场地设置
    FIELD_SIZE = (3.0, 3.0)  # 默认地图尺寸；每回合从FIELD_SIZE_RANGE采样
    FIELD_SIZE_RANGE = ((2.0, 4.0), (2.0, 4.0))
    SOURCE_CENTER_DISTANCE_RANGE = (0.5, 1.2)
    SOURCE_BOUNDARY_MARGIN = 0.3
    INITIAL_YAW_RANDOM = True
    GRID_RESOLUTION = 0.1  # 网格分辨率(米)

    # 机器人参数
    ROBOT_RADIUS = 0.15  # 机器人半径(米)
    MAX_LINEAR_VEL = 0.3  # 最大线速度(m/s)
    MAX_ANGULAR_VEL = 1.0  # 最大角速度(rad/s)

    # 速度映射
    VELOCITY_MAP = {
        'STOP': (0.0, 0.0),
        'FORWARD_SLOW': (0.1, 0.0),
        'FORWARD': (0.2, 0.0),
        'TURN_LEFT': (0.0, 0.8),
        'TURN_RIGHT': (0.0, -0.8),
        'ARC_LEFT': (0.15, 0.5),
        'ARC_RIGHT': (0.15, -0.5),
    }

    # 时间设置
    DT = 0.1  # 传感器内部更新步长(秒)
    STATE_UPDATE_FREQ = 10  # 状态更新频率(Hz)
    DECISION_FREQ = 2  # 决策频率(Hz)

    # 传感器参数 (方案E: 增加真实性)
    TGS2602_TAU = 3.0  # TGS2602一阶滞后时间常数(秒)
    TGS2602_RECOVERY_TAU = 20.0  # 恢复时间常数(秒)
    SENSOR_NOISE_STD = 0.08  # 传感器噪声标准差 (增加真实性)

    # 羽流模型参数 (方案E: 增加随机性)
    PLUME_DIFFUSION = 0.1  # 扩散系数
    PLUME_DECAY = 0.004  # 衰减系数
    PLUME_WIDTH = 1.0  # 基础宽度
    PLUME_SCALING = 30.0  # 幅值缩放
    WIND_TURBULENCE = False  # 第一版室内无风
    TURBULENCE_SCALE = 0.15  # 湍流强度
    PLUME_INTERMITTENCY = 0.8  # 间歇性(80%时间有信号)

    # 成功判定 (方案E: 放宽条件)
    SUCCESS_RADIUS = 0.25  # 源点确认半径(米)
    SUCCESS_THRESHOLD = 0.5  # 酒精概率阈值 (0.55→0.5)
    SUCCESS_WINDOW = 2  # 连续两个决策周期确认
    MAX_EPISODE_SECONDS = 60.0
    # Position-aware shaping: the policy is warned before the hard boundary,
    # and crossing it is a strong terminal penalty.
    BOUNDARY_WARNING_MARGIN = 0.30
    BOUNDARY_WARNING_PENALTY = 2.0
    BOUNDARY_PENALTY = 100.0
    BOUNDARY_SAFETY_ENABLED = True
    BOUNDARY_SAFETY_MARGIN = 0.05
    TEACHER_GUIDANCE_WEIGHT = 0.0
    TEACHER_MISMATCH_PENALTY = 0.0

    # PPO输入契约。位置特征由训练/评估入口显式打开，避免旧模型误读新状态。
    INCLUDE_GAS_RESISTANCE = True
    INCLUDE_POSITION = False


class ActionSpace:
    """动作空间定义"""
    ACTIONS = [
        'STOP',
        'FORWARD_SLOW',
        'FORWARD',
        'TURN_LEFT',
        'TURN_RIGHT',
        'ARC_LEFT',
        'ARC_RIGHT'
    ]

    @staticmethod
    def action_to_string(action_id):
        return ActionSpace.ACTIONS[action_id]

    @staticmethod
    def string_to_action(action_name):
        return ActionSpace.ACTIONS.index(action_name)
