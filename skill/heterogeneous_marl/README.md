# Heterogeneous MAPPO Research and Implementation Skill

## Purpose

Use this skill for three related but distinct tasks:

1. Reproduce and evaluate the submitted multi-agent MAPPO simulation in
   `提交/mappo/`.
2. Plan or implement algorithm and simulator extensions, with experiments that
   separate demonstrated behavior from hypotheses.
3. Guide developers or agents designing a real heterogeneous multi-vehicle
   system, while preserving local vehicle safety and clearly distinguishing
   completed simulation evidence from the completed assessment of physical
   deployment evidence.

The package in `提交/mappo/` is the source of truth for the current algorithm,
environment, checkpoints, and evaluation evidence. Do not use the older
`提交/marl/` implementation or recreate its wrappers. This skill does not
claim that MAPPO is deployed on multiple physical cars.

## Current algorithm and limits

- CTDE MAPPO: actors make decentralized decisions; training uses a
  centralized, permutation-invariant critic.
- A shared actor receives an 8-step history of 21-dimensional observations,
  role conditioning, received neighbor messages, and auxiliary state. It
  outputs one of seven motion actions and an 8-dimensional learned message.
- The simulator assigns `scout`, `tracker`, `confirmer`, and `relay` roles in a
  fixed cycle and associates them with different simulated capability values.
  This is not four separately trained policies, nor evidence of four physical
  vehicle types.
- The learned 8-D message is an internal simulation feature, not a validated
  network protocol for physical vehicles.
- Optional coverage priors can materially affect results. Always state when
  one is enabled; do not attribute a combined controller's performance to
  MAPPO alone.
- Simulation safety overrides are not a substitute for vehicle-local safety
  arbitration, watchdogs, command expiry, boundary protection, or emergency
  stop.

## Reproduce the submitted package

From the workspace root, run the package tests:

```bash
python3 -B 提交/mappo/test_mappo.py
```

Run a deterministic-policy evaluation using the frozen checkpoint:

```bash
python3 提交/mappo/evaluate_mappo.py \
  --checkpoint 提交/mappo/artifacts/mappo_max_response.pt \
  --scenario challenge_30m_10agents_10sources \
  --agents 10 --max-seconds 1500 --required-sources 3 \
  --episodes 20 --seed 20261030 \
  --output /tmp/mappo_eval.json
```

Compare a new run with the frozen records by aggregating the relevant files:

```bash
python3 提交/mappo/aggregate_evidence.py \
  提交/mappo/artifacts/current_normal3_20.json \
  /tmp/mappo_eval.json \
  --output /tmp/mappo_evidence_summary.json
```

For training options, inspect `python3 提交/mappo/train_mappo.py --help`.
Pass both `--checkpoint` and `--output` explicitly: the script defaults still
point to an older `MARL/new/` path. Keep new checkpoints/logs in a new run
directory under `提交/mappo/` or a designated external results directory; do
not overwrite the frozen checkpoint or evidence. Training and evaluation
require NumPy and PyTorch. The package README lists the project's preferred
environment and checksum verification command.

## Read the evidence correctly

Read `提交/mappo/docs/阶段性评估结果.md` and inspect the JSON artifacts before
making claims. At the frozen evaluation:

- 30 m, 10 agents, 10 static sources, target 3: 20/20 episodes succeeded.
- 30 m, 10 agents, 10 sources, target all 10 with a coverage prior and dynamic
  sources: 19/20 succeeded; the result is for the combined MAPPO-plus-prior
  controller.
- 50 m, 10 agents, 20 sources, target all 20 with a coverage prior: 16/30
  succeeded across the two reported batches.
- The reported 30 m three-source communication ablation achieved 20/20 in
  each group with communication on, off, and packet loss. This has no success
  rate separation and does not establish a stable benefit from communication.

These are finite simulation samples, not real-world migration guarantees.
Keep scenario, seeds, episode count, checkpoint, aggregation mode, coverage
prior, and all overrides with every result. Use independent seeds and report
uncertainty; do not select only favorable episodes.

## Extend toward real heterogeneous vehicles

Use `implementation_guide.md` for research and implementation workflow. The
JSON files under `contracts/` are design-only starting points for future
vehicle summaries and adapters. They are not the simulator's learned-message
schema and are not a deployed wire protocol.

For any physical extension, first define per-vehicle identity, capabilities,
local observation/action contracts, time and message freshness, task ownership,
and failure behavior. A fleet coordinator may allocate tasks or share compact
beliefs, but each vehicle must validate commands locally and remain safe when
coordination is delayed or unavailable. Do not start another control client
that competes with the existing single-car workflow.

## Evidence levels

Use the following labels in reports:

- `implemented_simulation`: code exists and bundled tests or runs exercise it;
- `reproducible_simulation`: an independent run reproduces a stated result;
- `design_only`: a proposed interface, architecture, or experiment without
  implementation evidence;
- `physical_single_vehicle`: measured on the current physical car;
- `physical_multi_vehicle`: measured with multiple physical vehicles under the
  stated network, safety, and task conditions.

Never promote a claim to a stronger level without the corresponding logs and
test conditions.
