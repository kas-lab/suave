# Metrics Reference

SUAVE automatically records mission metrics at the end of each run via the `mission_metrics` node (`suave_metrics` package). Results are written as CSV files to a configurable output directory.

## Output location

By default results are saved to `~/suave/results/`. This can be changed via the `result_path` ROS parameter or the `result_path` field in `runner_config.yml`.

Each run produces a main summary file plus up to three optional per-event files, all using the same base filename (`result_filename`).

## Main summary file — `<result_filename>.csv`

One row is appended per completed run.

| Column | Type | Description |
|---|---|---|
| `mission name` | string | Mission label — concatenation of `mission_name` and `adaptation_manager` parameters |
| `datetime` | string | Timestamp when results were saved (`DD-Mon-YYYY-HH-MM-SS`) |
| `initial pos (x,y)` | string | AUV spawn position in the Gazebo world frame |
| `mission duration (s)` | int | Elapsed time from GUIDED mode start to mission end |
| `pipeline found` | bool | Whether the pipeline was detected during the mission |
| `time searching pipeline (s)` | int | Time from mission start to pipeline detection (equals mission duration if not found) |
| `distance inspected (m)` | float | Total pipeline distance covered during inspection |
| `mean reaction time (s)` | float | Mean reaction time across all adaptation events (see below); 0.0 if no events occurred |

## Per-event files

These files are only created if at least one event of the corresponding type occurred. Each has columns: `mission_name`, `datetime`, `reaction time (s)`.

### `<result_filename>_component_recovery_time.csv`

One row per **thruster failure event**. Reaction time is measured from the moment a thruster is reported as failed in `/diagnostics` to when `f_maintain_motion_node` transitions back to `active` (indicating the managing subsystem issued the recovery reconfiguration).

### `<result_filename>_wv_reaction_time.csv`

One row per **water visibility change event**. Reaction time is measured from the moment the measured water visibility causes a mismatch with the current spiral altitude to when `f_generate_search_path_node` adopts the correct altitude (via parameter change on `/f_generate_search_path_node`).

### `<result_filename>_battery_reaction_time.csv`

One row per **low battery event**. Reaction time is measured from when battery level drops below the threshold (default 0.25) to when `generate_recharge_path_node` transitions to `active`.

## Notes on `mean_reaction_time`

The mean reaction time in the summary file averages across **all three** event types (thruster, water visibility, battery). When comparing managing subsystems, consider inspecting the per-event files to separate performance on each disturbance type.

## Mission config parameters

The following `mission_metrics` ROS parameters affect what is recorded:

| Parameter | Default | Description |
|---|---|---|
| `result_path` | `~/suave/results` | Output directory |
| `result_filename` | `mission_results` | Base filename (without `.csv`) |
| `adaptation_manager` | `none` | Label appended to `mission_name` in output |
| `mission_name` | `inspection` | Mission label |
| `water_visibility_threshold` | `[3.25, 2.25, 1.25]` | Water visibility thresholds for altitude selection |
| `expected_altitude` | `[3.0, 2.0, 1.0]` | Expected spiral altitudes corresponding to each threshold |
| `battery_limit` | `0.25` | Battery fraction below which a low-battery event is triggered |

The deprecated misspelling `water_visibiity_threshold` remains accepted for
backward compatibility, but new configurations should use
`water_visibility_threshold`.
