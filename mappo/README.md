# 异构多智能体 MAPPO

本目录包含基于 21 维单车 PPO 接口实现的异构多智能体 MAPPO 仿真算法、训练与评估脚本，以及阶段性模型和可复核评估记录。

## 方法概览

- 采用集中训练、分散执行（CTDE）：训练时使用集中式 Critic，执行策略由各智能体 Actor 产生。
- Actor 接收 8 步 × 21 维历史观测、角色条件和邻居消息，输出原有 7 类运动动作及 8 维学习消息。
- 支持角色条件、通信开关、丢包率和消息聚合方式实验。
- `marl/` 保存本仿真环境使用的动作、场景配置、观测与气味场模型，保证本目录代码可独立运行。

## 目录

- `train_mappo.py`：MAPPO 训练与 checkpoint 保存。
- `evaluate_mappo.py`：固定 checkpoint 的独立回合评估。
- `env.py`、`mappo_core.py`：仿真任务环境和 Actor/Critic 实现。
- `marl/`：算法仿真依赖模块。
- `aggregate_evidence.py`：汇总独立评估 JSON 并计算 Wilson 95% 区间。
- `test_mappo.py`：网络张量形状、前向计算及环境可复现性测试。
- `artifacts/`：冻结主模型及其对应的精选评估 JSON。
- `docs/阶段性评估结果.md`：冻结结果、配置和解释范围。
- `figures/`：中文 MAPPO 论文风格仿真评估图、可复核数据表、绘图脚本和图注说明。

## 环境与快速验证

需要 Python 3.10+、NumPy 和 PyTorch。使用项目虚拟环境时：

```bash
/home/aup/embodied/.venv/bin/python -B 提交/mappo/test_mappo.py
```

也可以在仓库根目录使用已有的 Python 环境：

```bash
python3 -m pip install numpy torch
python3 -B 提交/mappo/test_mappo.py
```

## 评估示例

```bash
/home/aup/embodied/.venv/bin/python 提交/mappo/evaluate_mappo.py \
  --checkpoint 提交/mappo/artifacts/mappo_max_response.pt \
  --scenario challenge_30m_10agents_10sources \
  --agents 10 --max-seconds 1500 --required-sources 3 \
  --episodes 20 --seed 20261030 \
  --message-aggregation max_response \
  --output /tmp/mappo_eval.json
```

结果汇总示例：

```bash
/home/aup/embodied/.venv/bin/python 提交/mappo/aggregate_evidence.py \
  提交/mappo/artifacts/current_normal3_20.json \
  提交/mappo/artifacts/current_normal10_prior20.json \
  --output /tmp/mappo_evidence_summary.json
```

训练脚本的完整参数见：

```bash
/home/aup/embodied/.venv/bin/python 提交/mappo/train_mappo.py --help
```

训练与评估均为仿真程序；本目录不包含车端控制程序，也不会连接或控制实车。阶段性指标及适用范围见 `docs/阶段性评估结果.md`。

文件完整性可在本目录运行 `sha256sum -c CHECKSUMS.sha256` 检查。
