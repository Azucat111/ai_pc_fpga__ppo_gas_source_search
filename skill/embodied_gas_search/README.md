# Embodied Gas-Source Search Skill

This skill is the physical single-vehicle workflow entry. It guides an
operator or development agent through preflight checks, model selection,
single-car workflow startup, evidence collection, and safe recovery.

The authoritative runtime remains:

```text
host/tools/car_workflow.py
```

The skill must not create a second UDP client, bypass the workflow, or send
motor commands directly. Live actions remain explicitly enabled by the
operator and are subject to the existing car-side safety path.

## Current evidence boundary

- The single-car PPO contract is the 21-dimensional input contract.
- BME688 training and evaluation are offline workflows in `bme688/`.
- The physical single-car workflow is the demonstrated embodied path.
- Multi-car operation is not validated by this skill; see
  `../heterogeneous_marl/` for MAPPO simulation research and staged future
  implementation guidance.

## Suggested operator sequence

1. Verify hardware, network, power and emergency-stop conditions.
2. Check telemetry freshness and sensor validity.
3. Select a validated model and matching preprocessing metadata.
4. Start the single workflow and record its status output.
5. Preserve task, communication, sensor and action evidence.
6. Use only the existing pause, stop, timeout-confirmation and recovery
   commands.

Any claim of a complete physical task must include the corresponding raw log,
hardware conditions, model metadata and failure/stop evidence.
