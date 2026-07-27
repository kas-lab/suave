# Repository Guidelines

## Project Overview

SUAVE (Self-adaptive Underwater Autonomous Vehicle Exemplar) is a ROS 2 Humble exemplar for a single AUV pipeline-inspection mission: search for a pipeline, follow it, and inspect it. The repository separates the managed subsystem (vehicle mission functionality) from managing subsystems (adaptation logic), so adaptation managers can be swapped through standard ROS 2 interfaces.

Runtime stack: ROS 2 Humble, Gazebo Harmonic, ArduSub/ArduPilot SITL, MAVROS, BehaviorTree.CPP, MROS2/Metacontrol, and Docker/Kasm.

## Project Structure & Module Organization

Core managed-system Python nodes live in `suave/suave/`, with launch files and sim config in `suave/launch/` and `suave/config/`. Top-level mission/manager composition is in `suave_bringup/`. Monitoring nodes are in `suave_monitor/`, missions and mission configs in `suave_missions/`, metrics in `suave_metrics/`, auxiliary tools in `suave_tools/`, and experiment orchestration plus statistical analysis in `suave_runner/`. Managing subsystems are under `suave_managing/`, including `suave_none`, `suave_random`, `suave_metacontrol`, and the C++ BehaviorTree.CPP package `suave_bt`. Custom services are defined in `suave_msgs/srv/`. Tests are usually in each package's `test/` directory. Docker assets are in `docker/`, runner scripts in `runner/`, and documentation in `docs/source/`.

## Build, Test, and Development Commands

Run anything related to SUAVE execution inside the `suave_runner` container, including tests, ROS launches, `colcon`, and direct `pytest` runs. The host machine is not assumed to have SUAVE or ROS dependencies installed. Use the container's default sourced workspace configuration; do not override `PYTHONPATH`, `ROS_LOG_DIR`, or similar ROS/Python environment variables unless the user explicitly asks. The exception is `suave_runner`'s `_run_launchfile`, which intentionally sets `ROS_LOG_DIR` per run so node logs are stored with experiment results.

Default container command pattern:

```bash
docker exec suave_runner bash -lc 'cd /home/ubuntu-user/suave_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && <command>'
```

Run ROS build and test commands from the ROS workspace root inside the container, `/home/ubuntu-user/suave_ws`, not from inside `src/suave`.

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
colcon build --symlink-install --executor sequential --parallel-workers 1
colcon build --symlink-install --packages-select <package_name>
source install/setup.bash
colcon test --packages-select <package_name> --event-handlers console_direct+
colcon test-result --verbose
python3 -m pytest -q <package>/test

# Test all SUAVE packages
colcon test --event-handlers console_cohesion+ --packages-select suave suave_base suave_bt suave_bringup suave_metacontrol suave_metrics suave_missions suave_monitor suave_msgs suave_none suave_random suave_runner suave_tools

# Auto-format C++ in the package being edited
ament_uncrustify --reformat
```

After changing package data, launch files, setup metadata, or installed config files, rebuild with `colcon build --symlink-install` so installed resources are refreshed.

## Running SUAVE

Run SUAVE from inside the `suave_runner` container using the default workspace environment.

Use `cd runner && ./example_run.sh` for a full example. For manual runs:

```bash
# ArduSub SITL in a separate terminal
sim_vehicle.py -L RATBeach -v ArduSub --model=JSON --console

# Simulation
ros2 launch suave simulation.launch.py x:=-17.0 y:=2.0

# Mission, defaulting to no adaptation manager
ros2 launch suave_bringup mission.launch.py

# Mission with a specific manager
ros2 launch suave_bringup mission.launch.py adaptation_manager:=bt result_filename:=measurement_1
```

Valid `adaptation_manager` values are `none`, `metacontrol`, `random`, and `bt`. The preferred campaign runner is `ros2 launch suave_runner suave_runner.launch.py`; its config is `suave_runner/config/runner_config.yml` and results default to `~/suave/results/`. The shell runner is `cd runner && ./runner.sh [true|false] [metacontrol|random|none|bt] [time|distance] <runs>`, with `headless_runner.sh` using `screen` instead of `xfce4-terminal`.

MAVROS default FCU URL is `udp://0.0.0.0:14550@14555`, avoiding the need for `sim_vehicle --out=...`.

## Docker

```bash
docker run -it --shm-size=512m -p 6901:6901 -e VNC_PW=password --security-opt seccomp=unconfined ghcr.io/kas-lab/suave:main
./build_docker_images.sh
docker build -t suave-headless:dev -f docker/dockerfile-suave-headless .
docker build --check -f docker/dockerfile-suave-headless .
docker build --check --build-arg BASE_IMAGE=kasm-jammy:dev -f docker/dockerfile-suave .
```

Dockerfiles are intentionally lowercase as `docker/dockerfile-*`. When checking `docker/dockerfile-suave` without a local base, GHCR may return `denied`; use `--build-arg BASE_IMAGE=kasm-jammy:dev` after building the base locally. GUI image results are under `/home/kasm-user/suave/results`; headless image results are under `/home/ubuntu-user/suave/results`. The headless image has no `CMD`, so pass the runner command explicitly on `docker run`.

Version pins live in `docker/versions.env`, and Python package versions live in repository-root `requirements.txt`. `build_docker_images.sh` sources `versions.env` and forwards SHAs as build args.

GitHub Actions must use full commit SHAs rather than mutable action tags. Keep a version comment beside each SHA for maintainability. In the container workflow, pass the digest emitted by the `kasm-jammy` build to the dependent SUAVE image build; do not consume the just-built base through `:latest`.

## Navigation and MAVROS Frames

`suave/config/suave_mavros_apm_config.yaml` sets `local_position.frame_id: map`, so `mavros/local_position/pose` is in the Gazebo/world frame. Do not add a coordinate conversion between Gazebo poses and MAVROS local positions.

`MavrosPositionController.has_local_pose` is the readiness gate before sending setpoints. The controller creates ROS entities through its owning lifecycle node and is not a separately spun node. Spiral search initializes its center from the robot's MAVROS local position on activation, not from the world origin.

## ROS Interfaces for Managing Subsystems

A compliant managing subsystem must interact with `/diagnostics` (`diagnostic_msgs/DiagnosticArray`), `/task/request` and `/task/cancel` (`suave_msgs/srv/Task`), and system_modes `ChangeMode` services: `/f_maintain_motion/change_mode`, `/f_generate_search_path/change_mode`, and `/f_follow_pipeline/change_mode`.

There are two ways to connect a managing subsystem. For **external packages**, include `suave_base/launch/suave_base.launch.py` (which starts the managed system and metrics with `task_bridge` disabled) and add your manager nodes alongside it — no changes to the suave repo are needed. For **built-in managers contributed upstream**, keep the manager launch file manager-only and wire it into `suave_bringup/launch/mission.launch.py` behind an `adaptation_manager` condition; `suave_bringup` owns the managed system, mission node, and metrics composition in that path.

## Runner and Metrics IPC

`mission_metrics/done` (`std_msgs/Bool`) signals completion of an experiment run. Both the `MissionMetrics` publisher and `ExperimentRunnerNode` subscriber must use `RELIABLE` reliability, `TRANSIENT_LOCAL` durability, and depth 1; incompatible QoS can silently prevent DDS matching.

Per-run output belongs under `<result_path>/logs/run_{exp_idx}_{run_idx}/`: `ardupilot.log` contains SITL output, while `simulation/` and `experiment/` contain ROS logs redirected through `ROS_LOG_DIR`. `resume_result_path` resumes an interrupted campaign from an existing results folder. `random_seed`, defaulting to `100`, controls reproducible perturbation sequences.

## Lifecycle Action Server Patterns

Lifecycle action servers are created in `on_configure()` so they remain discoverable. The `use_action_server` parameter defaults to `False`: legacy behavior starts from lifecycle activation when false, while action mode waits for an accepted goal when true. Use `make_goal_callback()`, `accept_cancel`, `use_action_server()`, and `lifecycle_state_is_active()` from `suave/suave/action_server_utils.py` instead of duplicating goal, cancel, lifecycle-state, or mode checks. Use `wait_for_action_completion()` before destroying resources used by an active execute callback. On ROS 2 Humble, `node._state_machine.current_state` is `(state_id, state_label)`; the shared active-state helper intentionally checks element `1` for the `active` label and centralizes use of this internal rclpy API.

Lifecycle action nodes use two threading events. `_abort_event` is created in `__init__`, cleared in `on_activate`, and set in both `on_deactivate` and `on_shutdown`; execute callbacks map it to goal abortion. `_goal_executing` is set at execute-callback entry and cleared in `finally`, allowing the shared goal callback to enforce a single active goal. `on_cleanup` must destroy the action server to avoid duplicate servers after a configure-cleanup-configure cycle.

Pass cancellation as a callable, for example `lambda: goal_handle.is_cancel_requested`. `is_cancel_requested` is a boolean property, so passing its current value captures a stale snapshot. When a node uses a stop-reason enum, represent lifecycle deactivation separately from client cancellation and map deactivation to `goal_handle.abort()`.

Keep action callbacks as policy wrappers around node-local core behavior. Core routines should return explicit success, failure, progress, or stop outcomes; action wrappers map those outcomes to `succeed()`, `abort()`, or `canceled()`, while lifecycle wrappers own task start/stop. Do not leave legacy timers or tasks running in action mode. For long-running operations, inject a stop policy and check it inside every setpoint/service wait loop so cancellation, lifecycle deactivation, and timeout remain effective. Do not remove an in-progress waypoint from a path until it has actually been reached.

Use `call_service_with_timeout()` from `suave/suave/ros_service_utils.py` for bounded service waits. The function is asynchronous at the ROS client layer even though it waits for completion; do not name wrappers `*_sync`. On Humble, the type returned by `Node.create_rate()` is `rclpy.timer.Rate`, not `rclpy.rate.Rate`.

Action-server tests should spin the helper node and node under test in separate executors via `suave/test/action_test_utils.py`; sharing an executor can cause re-entrant `spin_until_future_complete()` failures. Test terminal-state mapping and stops inside inner wait loops, not only goal acceptance. Mock goal handles need `is_cancel_requested = False` when callbacks wrap that property in a lambda. Monkeypatched core methods that accept `cancel_requested=None` must preserve that argument. The package test run may emit `rclpy.shutdown() has been called` after all tests pass; treat it as known stderr noise only when pytest and colcon report success. The `rclpy.action` module is provided by the ROS package `rclpy`; never add a `rclpy_action` rosdep dependency.

Put new action definitions in `suave_msgs/action/`. Add `action_msgs` to `suave_msgs/CMakeLists.txt` and `suave_msgs/package.xml`. If a node class and action type share a name, alias the action import to avoid collisions.

Vehicle position handling uses the non-node `MavrosPositionController`, which creates its subscription and publisher through the owning lifecycle node. Keep its local-position subscription in a callback group separate from blocking behavior callbacks and run these nodes in a `MultiThreadedExecutor`. Do not reintroduce a separately spun child node. Lifecycle nodes own the `ground_depth_gz` and altitude parameters, pass altitude explicitly to the controller, and destroy configured controller resources only after active callbacks have stopped.

## Coding Style & Naming Conventions

Python packages use `ament_python`, `rclpy`, four-space indentation, snake_case modules/functions/parameters, and PascalCase classes. Python code must pass flake8 and pep257. Prefer concise single-line docstrings because the repository's PEP 257 configuration rejects the alternate multiline summary layout. Public production modules, packages, classes, constructors, methods, entry points, and launch-description functions require docstrings; tests and private nested helpers do not need docstrings unless their behavior is otherwise unclear. Nested functions need the blank-line separation required by E306. Add this Apache-2.0 header to Python files:

```python
# Copyright 2026 KAS Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
```

Nodes subclass `rclpy.node.Node`, declare/read ROS parameters in `__init__`, expose module-level `main()` entry points, and register console scripts in `setup.py`. Use `extras_require={'test': ['pytest']}` rather than deprecated `tests_require`. Keep imports PEP8/flake8-clean and add pep257 docstrings for new public modules, classes, and functions. Avoid type hints unless they add clear value to the surrounding code.

Package metadata must declare `Apache-2.0` consistently in both `setup.py` and `package.xml`. New Python packages need an enabled `test/test_copyright.py`; do not leave the generated skip marker after headers are in place. In CMake packages, `set(ament_cmake_copyright_FOUND TRUE)` disables copyright checking and should be removed once source headers are compliant.

C++ code in `suave_bt` targets C++17 with `-Wall -Wextra -Wpedantic`; package headers live under `include/suave_bt/`, implementations under `src/suave_bt/`, BT node names are registered in `src/suave_bt.cpp`, and XML trees live in `bts/`. `suave_msgs` is an `ament_cmake` package.

Launch files are Python ROS launch descriptions installed via `glob` in `setup.py` or CMake. Keep MAVROS launch args such as `fcu_url`, `gcs_url`, `mavros_config_yaml`, and `mavros_pluginlists_yaml` overrideable. New substantive files should follow nearby Apache-2.0 header style where present.

## Testing Guidelines

Python tests use `pytest` plus ROS linters such as `ament_flake8`, `ament_pep257`, and `ament_copyright`. Name tests `test_*.py` and place them in the affected package's `test/` directory. For Python changes, run `colcon test --packages-select <pkg> --event-handlers console_direct+` when ROS dependencies are available. For CMake, C++, or message changes, build the affected package before testing. Note any skipped tests when Gazebo, ArduSub, MAVROS, Docker, or other simulator dependencies are unavailable.

Use targeted checks when applicable:

```bash
python3 -m pydocstyle <changed-python-files-or-directories>
python3 -m flake8 <changed-python-files-or-directories>
python3 -m py_compile <launch_file.py>
bash -n <script>
docker build --check -f <dockerfile> .
```

If `setup.py` `data_files` or console script entry points change, verify they are installed correctly after build.

## Documentation

`docs/source/` is a Sphinx GitHub Pages site that mirrors README content. Keep both in sync when changing installation steps, Docker commands, runner usage, architecture, extension guidance, implementations, metrics, troubleshooting, related work, citing, or API docs. When changing `suave_runner` behavior or parameters, update both `suave_runner/README.md` and `docs/source/run.md`.

The VCS dependencies file is `suave.repos`. The old name `suave.rosinstall` is obsolete and should not be referenced.

## Commit & Pull Request Guidelines

Recent commits use short imperative subjects, for example `add statistical_analysis` or `pass mission_config to task_bridge_none`; an emoji prefix appears occasionally for fixes. Keep commits focused and mention the affected package when useful. Pull requests should describe behavior changes, list test commands and results, link related issues, and include screenshots or logs for simulator, UI, Docker, or mission-run changes.

## Agent-Specific Instructions

Before editing, check `git status --short` and preserve unrelated user changes. Prefer `rg` for searches and keep changes scoped to the relevant ROS package. For SUAVE validation, always run tests and runtime checks inside `suave_runner` with the container's default sourced environment. Before finishing, run `git status --short` again and distinguish your edits from pre-existing uncommitted experiment, simulator, or Docker changes. Do not revert unrelated work.
