# AMD FPGA Gas-Source Search Code

This is a compact source-code package for the AMD FPGA embodied gas-source search project.

## Contents

- `fpga/src/`: SystemVerilog RTL, Vivado build files, board-side software, firmware, vendor source, and RTL tests.
- `host/`: AI PC runtime, safety and protocol integration, configuration, and tests.
- `ppo/`, `mappo/`: training, simulation, evaluation, and plotting source code.
- `bme688/`: sensor data collection, preprocessing, training, inference, and tests.
- `skill/`: workflow and interface reference material.

Datasets, bitstreams, build outputs, measurement evidence, figures, and reports are not included. The primary PPO, BME688, and MAPPO checkpoints are retained. BME688 inference also requires its scaler and model metadata, which are included beside its checkpoint. Other runtime or training workflows may require separately collected data or generated artifacts.
