# 21-D PPO odor-source localization

This directory contains the 21-dimensional PPO policy used by the single-car
simulation and the reproducible evaluation/training code. The policy is a
PC-side decision model. Boundary protection and vehicle safety remain outside
the policy.

## Package contents

- `models/best_model.pth`: submitted 21-D checkpoint.
- `models/ppo_network.py`: network and PPO agent implementation.
- `environment/`: deterministic simulation environment and sensor model.
- `tools/evaluate_ppo_v3.py`: independent evaluation with optional boundary safety.
- `tools/evaluate_ppo_transfer.py`: transfer/scaling evaluation entry point.
- `train_ppo.py`: training entry point. Training outputs must be written to a
  separate working directory and are not submission artifacts.

The observation is an 8-step history with 21 features: the base gas/motion
features plus normalized relative position and heading representation. The
action interface has seven discrete actions. The submitted checkpoint should
be treated as a candidate policy until the physical end-to-end acceptance test
is completed.

## Environment

Install the dependencies from this directory:

```bash
cd /home/aup/workspace/amd_project/提交
python3 -m pip install -r ppo/requirements.txt
```

CPU evaluation is supported. CUDA/ROCm is optional and must be verified on the
target machine before use.

## Evaluate the submitted checkpoint

```bash
cd /home/aup/workspace/amd_project/提交
python3 ppo/tools/evaluate_ppo_v3.py \
  --feature-count 21 \
  --checkpoint ppo/models/best_model.pth \
  --episodes 1000 \
  --seed-offset 500000
```

Use `--no-boundary-safety` only for an explicit policy-only comparison. The
normal evaluation keeps the independent boundary safety layer enabled.

## Reproduce publication figures

```bash
cd /home/aup/workspace/amd_project
/home/aup/embodied/.venv/bin/python 提交/ppo/tools/plot_ppo_evaluation.py \
  --episodes 1000 --seed-offset 500000
```

The script writes vector PDF/SVG, 600-DPI PNG, per-episode CSV and a JSON
provenance/summary manifest to `提交/ppo/figures/`. The figure summarizes a
deterministic-policy simulation evaluation with the boundary safety state
machine enabled; it is not a training curve or physical-car result. Its 95%
interval is the Wilson binomial interval for the observed episode success
fraction. The successful-episode length panel shows the empirical ECDF and
interquartile range.

## Train a new policy

```bash
cd /home/aup/workspace/amd_project/提交
PPO_CHECKPOINT_DIR=/tmp/ppo_training_output \
  python3 ppo/train_ppo.py
```

The training script uses its configured simulation defaults. Inspect the
script and environment configuration before changing the feature contract.
Generated checkpoints, logs and plots belong in the external output directory
and must not be copied back into this release package without review.

## Limitations

Simulation success is not physical-car validation. The package still requires
an integrated evidence record for sensor sampling, PC inference, state-machine
decision, command transmission and vehicle response latency.
