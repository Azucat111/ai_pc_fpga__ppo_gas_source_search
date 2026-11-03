# BME688 data and model

This folder contains six guided GAS10 sessions (air/alcohol), the current experimental model, three-fold multi-session test outputs, and training utilities. The collected sessions are from one device and a narrow time/environment range; results do not establish cross-device or general environmental performance.

## Reproduce the current session-based training

From the package root, install `bme688/requirements.txt`, then run:

```bash
python3 bme688/training/train_air_alcohol.py
```

The defaults train on the listed air/alcohol sessions and evaluate on disjoint sessions. Results are written to `bme688/model/new_session_run/`. To use other guided sessions, supply `--data-dir`, `--train-files`, and `--eval-files`; both splits must contain labels `air=0` and `alcohol=1` and must not share files.

To train/evaluate from the Dashboard-format labeled session files with the cleaning and comparison pipeline:

```bash
python3 bme688/training/collect_bme688_data.py \
  --data-dir bme688/data \
  --output-dir bme688/model/new_run
```

The model uses `R0`-`R9`, per-row mean normalization, then a `StandardScaler` fitted only on training data. Keep the generated model, scaler, metadata, and matching `models/` architecture together.

The 2026-11-01 multi-session evaluation is stored under `bme688/multisession/20261101_210452/`. It uses three complete air sessions and three complete alcohol sessions with complete-session holdout folds. This is a same-day, same-device result and is not cross-date, cross-environment, cross-device, or final deployment validation.

`prepare_air_alcohol_dataset.py` is for a separate cleaned dataset that contains `class_key` and `label`; its input and output paths are required because that external dataset is not bundled. `predict_air_alcohol.py` is a live PYNQ stream client and is not needed for offline training or tests; do not run it unless intentionally connecting to a board.
