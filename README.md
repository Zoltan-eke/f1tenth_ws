# F1TENTH Vehicle Dynamics Simulation & Tire Model Calibration

ROS2 workspace built for an F1TENTH-scale (1:10) autonomous racing vehicle. The project simulates vehicle dynamics with a nonlinear Pacejka tire model, calibrates model parameters against logged rosbag data via grid search, and prepares the pipeline for CARLA-based validation.

Originally developed as a university thesis project (graded, completed). Published here as a portfolio piece — see [Known Limitations](#known-limitations) below for an honest account of the methodological gaps, and [Resolved in v2-cleanup](#resolved-in-v2-cleanup) for what's been fixed since.

## What this project does

- Ackermann-steered vehicle model (URDF/Xacro) and keyboard teleop for manual driving in simulation.
- Two tire models: a linear baseline and a nonlinear Pacejka model, with grid-search calibration of tire and geometry parameters (`B`, `C`, `C_lin`, wheelbase `lf`/`lr`).
- Data logging pipeline: raw and filtered odometry, IMU, TF, and Ackermann drive commands, recorded from simulation runs and replayed for offline analysis.
- Offline validation (`offline_validation.py`) comparing model output against logged trajectories using sliding-window RMSE.
- Groundwork for CARLA integration (leader/follower vehicle setup, message bridging) — not yet complete, out of scope for the current cleanup pass.

## Repository structure

```
src/
├── car_description/    # URDF/Xacro model, meshes, RViz config, Ackermann control params
├── car_simulation/     # Vehicle dynamics node (Pacejka model), sim launch files, drive replay
├── data_logger/        # Logging nodes, grid-search calibration scripts, offline validation
├── carla_msgs/         # CARLA message definitions (prep work, not integrated yet)
└── ros-bridge/         # CARLA ROS bridge (prep work, not integrated yet)
```

Key scripts in `data_logger/scripts/`:
- `grid_search_tire.py`, `grid_search_BC.py`, `grid_search_lf_lr.py` — parameter calibration via grid search over logged data.
- `offline_validation.py` — RMSE-based model validation against logged odometry.
- `filter_csvs.py`, `filter_drive.py` — preprocessing of raw logged CSVs.

## Methodology / workflow

The pipeline was built in stages, each one gating the next:

1. **Vehicle model declaration.** The vehicle was partially declared via URDF (`car_description`), enough to visualize it in RViz.
2. **RViz-based sanity check against the rosbag.** The declared model was checked visually against the recorded rosbag motion to confirm the vehicle stayed "on track." Once a stable, trustworthy segment of the recording was identified, the rosbag was cut down to that segment.
3. **Logging.** The `data_logger` nodes recorded the trimmed segment (odometry, IMU, TF, drive commands) into the raw/filtered CSVs used for calibration and validation.
4. **Modeling.** The linear and Pacejka tire models were built on top of the logged data, with the eventual goal of a working drive script that could control the vehicle using the calibrated model.

This staged approach is why the validation gap described below exists: the RViz check confirmed the vehicle *model* tracked plausibly, but the tire-model calibration itself was never re-validated against that same raw position data — it was validated against the linear model's synthetic output instead.

## Setup

Originally developed and tested on ROS2 Humble (Ubuntu 22.04). The `v2-cleanup` branch has also been built and verified under ROS2 Jazzy (Ubuntu 24.04) — both should work; if you hit distro-specific issues on one, try the other.

If running under WSL2 on Windows, use a native ext4 filesystem path (e.g. your Linux home directory) rather than `/mnt/c/...` — building under the Windows-mounted filesystem is significantly slower.

```bash
# clone into your ROS2 workspace src folder
cd ~/f1tenth_ws
colcon build --symlink-install
source install/setup.bash
```

CARLA integration (`carla_msgs`, `ros-bridge`) requires a separate CARLA server installation and is not required to run the core simulation/calibration pipeline.

## Resolved in v2-cleanup

A post-thesis technical review (branch `v2-cleanup`) found and fixed several bugs that were silently degrading the calibration and validation results below — not just style/dead-code issues, but ones that changed actual numeric output:

- **Dead linear tire-force branch.** `offline_validation.py`'s `dynamics()` computed the linear tire force (`C_lin`) but never used it — only the nonlinear Pacejka branch ran, regardless of slip angle. Re-enabled the intended threshold blend (linear below 0.1 rad slip, Pacejka above).
- **`Cf`/`Cr` name mismatch.** `params.yaml` declared `C_f`/`C_r`, but every node that reads this config (`ackermann_to_odom.py`, `vehicle_model_node.py`) looks for `Cf`/`Cr`. The keys were never actually read — every run silently used the hardcoded defaults instead of the calibrated value. This is a real behavior change, not a rename.
- **YAML-merge bug in all three grid search scripts.** The temporary per-iteration config passed to `offline_validation.py` was built with a flat `setdefault('ros__parameters', {})` instead of navigating into the real nested structure. As a result, every grid search iteration silently validated against default `mass`/`I_z`/wheelbase values instead of the calibrated master config — the search was effectively fitting against the wrong vehicle. Fixed the nesting and switched to `deepcopy` to prevent cross-iteration mutation.
- **Sensor timestamp source.** `save_tf.py` and `save_ackermann_cmd.py` stamped logged data with callback wall-clock receipt time instead of the message's own `header.stamp` (unlike `save_odom.py`/`save_imu.py`, which were already correct). This is a plausible contributing source of the sensor timestamp desync flagged as a limitation in the original thesis — it doesn't rule out other contributing factors (e.g. at the original recording stage).
- **Grid search boundary warnings.** All three grid search scripts now warn when the chosen optimum lands on the edge of its search range. Re-running the corrected pipeline still shows `B`, `D`, and the wheelbase total length (`L_total`) landing at their range boundary — the front/rear split (`LF_frac`) itself converged inside its range. This is a genuine finding, not an artifact of the earlier bugs, and the search ranges should be widened in a follow-up pass.
- **Unified scoring.** `grid_search_lf_lr.py` now uses the same `0.5*pos + 0.5*yaw` combined score as the other two scripts, instead of optimizing yaw error alone.
- Removed dead/unreachable files (`old.launch_v00.py`, `analyze.py`, `plot_odom.py`, `save_joint_states.py`) and a launch file that wrote conflicting output CSVs (`car_simulation_launch.py`, superseded by `sim_validation.launch.py` + `filter_drive.py`).

## Known limitations

These are still open — the fixes above didn't address them, and they matter for interpreting the current results correctly.

- **Validation ground truth is synthetic.** The Pacejka (nonlinear) tire model is validated against odometry generated by the linear tire model, not against real vehicle position data. This means the calibration confirms internal consistency between the two models rather than real-world accuracy. Validating against the original rosbag position data directly (`tf.csv`/`odom.csv`) is planned as the next phase.
- **Sequential (coordinate-descent) parameter optimization.** Tire parameters (`B`/`C`), linear parameters (`C_lin`/`D`), and geometry (`lf`/`lr`) are optimized one group at a time rather than jointly. Since geometry changes affect slip angle, which feeds back into tire parameter fit, the result is likely a local rather than global optimum. A joint or iterative-until-convergence approach is planned as a later pass, deliberately deferred to keep this cleanup scoped to concrete, verifiable bug fixes.
- **`B`, `D`, and the wheelbase total length (`L_total`) still converge at their search-range boundary**, even after the calibration-input bug fix above — now surfaced explicitly by the new boundary warnings. This suggests the true optimum lies outside the currently tested range for those parameters (the front/rear wheelbase split, `LF_frac`, did converge inside its range).
- **Limited excitation in the logged data.** Available rosbag recordings cover gentle ellipse and figure-8 driving patterns, which don't strongly excite the nonlinear slip regime the Pacejka model is meant to capture. Calibration results should be read with this in mind until higher-slip maneuvers are recorded.

## Status

**Phase 1 cleanup complete** (`v2-cleanup` branch): dead-code removal, parameter-loading bug fixes, unified grid-search scoring, and boundary-warning diagnostics. The original graded thesis submission is preserved unmodified at tag `v1.0-thesis-submission`.

Planned follow-up:
- **Phase 2:** validate the Pacejka model against real trajectory data instead of synthetic linear-model output, and address the rosbag's leading noise (vehicle was carried before being set down).
- **Phase 3:** joint (non-sequential) parameter optimization to address the coordinate-descent limitation above.
- **CARLA integration** (leader/follower, message bridging) — not started, out of scope for this pass.
