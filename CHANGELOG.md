
# Change Log
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## 1.5.0

### Added

1. ROS action servers for all four lifecycle nodes (`spiral_search`, `follow_pipeline`, `recharge_battery`, `recover_thrusters`). A `use_action_server` parameter (default `false`) keeps the existing lifecycle-transition-driven behavior; when `true`, each node waits for an action goal before starting its behavior.

2. New action message types in `suave_msgs`: `SpiralSearch`, `FollowPipeline`, `RechargeBattery`, `RecoverThrusters`.

3. Action client support in `suave_bt`: the BT nodes `search_pipeline`, `inspect_pipeline`, `recharge`, and `recover_thrusters` can now operate in either legacy mode (polling lifecycle transitions) or action-server mode (sending ROS goals). Controlled by a `use_action_server` launch argument that propagates to both the managing BT node and the managed lifecycle nodes.

4. `suave_runner`: experiment logs are now saved per-run under `<result_path>/logs/run_{exp_idx}_{run_idx}/`.

5. `suave_runner`: crashed campaigns can be resumed from an existing result folder via the `resume_result_path` parameter.

6. `suave_runner`: statistical analysis module, including matched-run Wilcoxon signed-rank comparisons (with optional Holm-Bonferroni correction), Q-Q plots for paired results, LaTeX table generation, and a results-summarization script.

7. `suave_runner`: batch runner (`run_batch`) that sequences multiple experiment campaigns (`exp1`-`exp3`, `extended_exp1`-`extended_exp3`) in one run, checkpointing progress to `state.json` so a batch can be resumed with `resume_state_file`.

8. Randomized experiment configuration: initial vehicle position, thruster failure events, and water-visibility shift timing can now be randomized, reproducibly, via a `random_seed` runner parameter.

9. Repository contributor guide.

10. Public Python API documentation.

### Changed

1. Removed `BlueROVGazebo` nested ROS node from lifecycle nodes. Navigation is now handled by a lightweight `MavrosPositionController` owned by each lifecycle node — no child node or private executor. The `suave` package no longer depends on `mavros_wrapper`.

2. `mavros/local_position/pose` is now published directly in the Gazebo/map frame. Frame offset conversions have been removed from all navigation nodes.

3. `suave_runner` no longer uses a `.done` file for run-completion signaling.

4. BehaviorTree.CPP is now built from source, pinned to version 4.10.0, instead of relying on the system package.

### Removed

1. Unused `const_dist_mission` mission type and its launch file.

### Fixed

1. `result_filename` argument not propagated correctly in launch files.

2. Runner crash when no thruster events were recorded in a run.

3. `suave_bt`'s `ChangeMode` node now verifies the target node's actual lifecycle state (and, for spiral search, its altitude parameter) before reporting a mode change as successful, instead of trusting a cached mode string. This fixes silent desyncs where a `system_modes` transition was lost but the behavior tree continued as if it had happened.

4. `suave_runner` detects known-broken runs after the fact (e.g. a lifecycle node that configured but never activated) and re-queues them instead of checkpointing bad data.

5. `follow_pipeline`'s inspected-distance tracking and resume/recharge handling, so distance accumulates correctly across pauses, recharge interruptions, and reactivation.

6. Water-visibility monitor now publishes an initial reading before the vehicle enters `GUIDED`, without letting startup time shift the configured visibility schedule.

## 1.4.0

### Added

1. New suave_runner package containing a new python-based runner script

2. A new docker image without vnc, allowing it to be executed without the web interface

3. Move to gazebo harmonic

## 1.3.0

### Added

1. Added battery monitor node [PR #148]

2. Added recharge battery task [PR #148]

4. Added battery_constraint argument to mission [PR #155]

5. Added qa_comparison_operator to water_visibility in suave.owl

6. Documentation with sphinx

7. CI to build sphinx documentation

8. Added mission metrics node [PR #148]

9. Added a behavior tree as a managing subsystem for SUAVE and SUAVE extended to serve as a baseline for comparison [PR #160]

10. Added reaction time metrics to measure how long the managing system takes to react and adapt the system [PR #161]

### Changed

1. Ardusub, mavros, ros_gz, mc_mros_reasoner, mc_mdl_tomasys, mros_ontology verions

2. Removed unused code

3. Mission config default parameters

4. Refactored repository. Created new packages : suave_monitor, and suave_metrics. Moved suave_metacontrol under suave_managing [PR #158]

5. Removed suave_reasoner. The Analyze logic was moved to metacontrol. [PR #169]

### Fixed

1. README.md

2. Fix task bridge callbacks

## 1.2.1 [SEAMS Publication]

### Fixed

1. Minor bugs

## 1.2.0 [SEAMS Publication]

Getting repository camera-ready

### Added

1. Added github action to build the docker images automatically, and push it to the repository registry

### Changed

1. Refactor mission and mission bridge to match the paper description
  * [Issue 117](https://github.com/kas-lab/suave/issues/117)


2. Upgraded mavros, mavros_wrapper, and mros versions

3. Refactor dockerfiles

4. Moved files within the repository

5. Updated README

### Fixed
