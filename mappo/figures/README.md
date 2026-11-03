# MAPPO 仿真评估图

## 图表说明

`mappo_evaluation.pdf`、`mappo_evaluation.svg` 和 `mappo_evaluation.png` 展示异构多智能体 MAPPO 在气味源定位仿真中的阶段性评估。每个点表示对应配置的成功回合比例，水平误差线为按逐回合二项结果计算的 Wilson 95% 置信区间；右侧标注成功数/回合数和区间。主评估与半径敏感性分组分开呈现，不应把这些配置视为同一实验条件。

- **A｜主评估：**30 m 场景、找到至少 3 个静态源：20/20；30 m 动态源全量任务，MAPPO 与覆盖先验组合：19/20；50 m 全量迁移压力任务，MAPPO 与覆盖先验组合：两份配置一致的评估记录合计 16/30（分别为 12/20 和 4/10）。
- **B｜配对半径敏感性：**50 m 迁移任务下，成功半径 1.5 m 为 10/20，2.0 m 为 11/20。该差异仅来自一个回合，不能据此得出半径调整带来稳定收益的结论。

**图注建议：** 异构多智能体 MAPPO 气味源定位策略的仿真评估。点为成功回合比例，误差线为二项比例 Wilson 95% 置信区间，标签给出成功数/总回合数。主评估包括 30 m 三源目标、30 m 十源全量覆盖（含覆盖先验）和 50 m 二十源迁移压力任务；迁移压力结果汇总两份相同配置记录。下方单独展示 50 m 场景的成功半径配对敏感性评估。各任务目标和场景不同，成功率不作跨配置直接比较。所有结果均为仿真，不代表真实传感器、PYNQ/Zynq 或实体车辆验证。

## 文件与数据

- `mappo_evaluation.pdf`：单页矢量图，适用于论文排版和缩放。
- `mappo_evaluation.svg`：矢量图，文本保留为文本；其他系统若缺少 Noto Sans CJK SC 字体，显示可能不同。
- `mappo_evaluation.png`：600 DPI 元数据的预览图；DPI 元数据不代表额外图像细节。
- `mappo_success_rates.csv`：作图使用的成功数、分母、成功率、Wilson 区间和来源文件。
- `mappo_evaluation_manifest.json`：统计口径、来源及汇总值机器可读记录。
- `plot_mappo_evaluation.py`：从原始逐回合 JSON 重新汇总、制图和导出上述文件。

CSV 和 manifest 在脚本运行时生成。迁移压力任务仅合并 `current_transfer20_confirm1_threshold075_20.json` 与 `current_transfer20_confirm1_threshold075_10b.json`，因为二者的场景和任务配置一致；样本量不等的两次评估合并后为 30 回合，来源仍在数据文件中分别列明。其他配置不合并。区间采用标准双侧 95% Wilson score interval；脚本没有平滑或插值数据。

## 重绘

在仓库根目录执行：

```bash
/home/aup/embodied/.venv/bin/python 提交/mappo/figures/plot_mappo_evaluation.py
```

需要 Python、NumPy、Matplotlib 以及 Noto Sans CJK SC 字体。脚本只读取 `提交/mappo/artifacts/` 下的评估 JSON，不运行模型、不修改原始记录或 `CHECKSUMS.sha256`。

## 解释边界

这些数值仅描述记录中的仿真任务和回合样本。尤其是使用覆盖先验的结果属于“MAPPO + 覆盖先验”组合，不能归因于纯 MAPPO。50 m 迁移任务的统计显示当前配置的样本结果，不等同于跨场景泛化结论。半径敏感性样本有限，也没有显示稳定收益。图表不能证明多车实物通信、FPGA/Zynq 实时硬件贡献、安全降级或物理闭环已验收。

图表已按通用出版格式完成导出与数据口径核对。不同期刊的版式要求存在差异，本图不绑定特定期刊模板。

## 制图方法参考

本图的证据保留、统计口径标注和出版图审阅参考 Scientific Visualization skill 的工作流：Kassis, T., Agarwal, V., He, Y., Patel, D., & Brueckner, A. M. (2026). *Scientific Agent Skills: A Library of Procedural Knowledge for Research Agents*. arXiv:2609.00065. https://doi.org/10.48550/arXiv.2609.00065
