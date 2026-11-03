# PPO 仿真评估图

## 图表内容

`ppo_evaluation.pdf`、`ppo_evaluation.svg` 和 `ppo_evaluation.png` 展示提交的 PPO 气味源定位策略在仿真环境中的一次确定性策略评估。策略以贪心方式选择动作（`argmax`），本次评估启用了独立的边界安全层；总计 1,000 个回合，种子从 500000 开始逐回合递增。

- **A｜回合结果：**成功 935/1,000（93.5%），超时 65，碰撞 0。误差线为成功率的 95% Wilson 二项比例置信区间（91.8%–94.9%）。条形颜色同时由图例和文字计数说明。
- **B｜成功回合步数分布：**仅纳入 935 个成功回合，曲线为经验累积分布函数（ECDF）；虚线标示中位数 43 步，阴影标示第 25 至第 75 百分位（29–62 步）。步数指环境中的决策步，不是秒数。

**图注建议：** PPO 气味源定位策略在确定性贪心推理、边界安全层开启条件下的仿真评估结果。A，1,000 个评估回合的终止结果及成功率 Wilson 95% 置信区间；B，成功回合（n=935）的决策步数经验累积分布，中位数及四分位距如图所示。该结果仅描述当前仿真配置，不代表真实传感器、PYNQ/Zynq 或实体车辆上的验证结果。

## 文件与数据

- `ppo_evaluation.pdf`：单页矢量图，适合论文排版和后续缩放。
- `ppo_evaluation.svg`：可编辑矢量图；SVG 文本使用本机 Noto Sans CJK SC 字体，其他系统若缺少该字体，显示可能不同。
- `ppo_evaluation.png`：高分辨率预览图（约 601 DPI 元数据）；DPI 元数据本身不等于额外图像细节。
- `ppo_episode_results.csv`：逐回合评估结果；每行一个仿真回合，含终止类型、成功标记、决策步数及安全覆盖次数。
- `ppo_evaluation_manifest.json`：评估规模、随机种子、终止计数、置信区间及模型检查点 SHA-256 等摘要信息。

图中的比例、置信区间和 ECDF 均由逐回合 CSV 直接计算。Wilson 区间按成功回合数与总回合数计算；ECDF 将成功回合步数升序排列后，以 `i / n_success` 作为累计比例。没有对成功步数进行平滑、插值或截断。

## 重绘与复现

仅调整中文标签或版式时，可从已有 CSV 重绘，不重新运行策略评估，也不会覆盖 CSV 或 manifest：

```bash
cd /home/aup/workspace/amd_project
/home/aup/embodied/.venv/bin/python 提交/ppo/tools/plot_ppo_evaluation.py \
  --from-csv 提交/ppo/figures/ppo_episode_results.csv \
  --output-dir 提交/ppo/figures
```

重新执行完整评估并更新图、逐回合 CSV 与 manifest：

```bash
cd /home/aup/workspace/amd_project
/home/aup/embodied/.venv/bin/python 提交/ppo/tools/plot_ppo_evaluation.py \
  --checkpoint 提交/ppo/models/best_model.pth \
  --feature-count 21 --episodes 1000 --seed-offset 500000
```

具体依赖版本见 `提交/ppo/requirements.txt`。绘图脚本使用 Matplotlib；中文字体为 Noto Sans CJK SC。本图已按通用论文插图规范完成导出；其尺寸、字体和格式不代表针对某一特定期刊的版式合规审查结果。

## 解释边界

这些数值是模拟环境内的结果，不是实体机器人成功率，也不能单独证明端到端具身闭环、传感器泛化或硬件安全。更换检查点、特征配置、环境参数、种子区间或边界安全设置后，必须重新评估，并相应更新 CSV、manifest、图和本说明中的统计值。训练曲线与独立场景/采集 session 泛化测试不包含在本图中。

## 制图方法参考

本图的证据保留、统计口径标注和出版图审阅遵循 Scientific Visualization skill 的工作流。参考：Kassis, T., Agarwal, V., He, Y., Patel, D., & Brueckner, A. M. (2026). *Scientific Agent Skills: A Library of Procedural Knowledge for Research Agents* (version 2). arXiv:2609.00065. https://doi.org/10.48550/arXiv.2609.00065
