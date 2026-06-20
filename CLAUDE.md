# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

SUAVE (Self-adaptive Underwater Autonomous Vehicle Exemplar) models a single AUV performing a pipeline inspection mission: searching for a pipeline, following it, and inspecting it. The repo cleanly separates a **managed subsystem** (vehicle mission functionality) from **managing subsystems** (adaptation logic), so different adaptation managers can be plugged in through standard ROS 2 interfaces.

Runtime stack: ROS 2 Humble · Gazebo Harmonic · ArduSub/ArduPilot SITL · MAVROS · BehaviorTree.CPP · MROS2/Metacontrol · Docker/Kasm.

## Repository Layout

| Path | Purpose |
|---|---|
| `suave/` | Managed subsystem: functionalities, launch files, sim config |
| `suave_monitor/` | Monitor nodes (thruster, battery, water visibility) — publish diagnostics |
| `suave_missions/` | Mission planners and launch files |
| `suave_metrics/` | Metrics collection |
| `suave_runner/` | Experiment runner and statistical analysis |
| `suave_tools/` | Auxiliary tools (PlotJuggler config, etc.) |
| `suave_msgs/` | Custom ROS service definitions (`Task.srv`, `GetPath.srv`) — `ament_cmake` |
| `suave_managing/suave_none/` | No-manager launch variant |
| `suave_managing/suave_random/` | Random managing subsystem |
| `suave_managing/suave_metacontrol/` | MROS2/Metacontrol managing subsystem |
| `suave_managing/suave_bt/` | BehaviorTree.CPP managing subsystem (C++17) |
| `docker/` | Dockerfile definitions and install scripts |
| `runner/` | Shell scripts for running experiments |

## Build & Test

Run anything related to SUAVE execution inside the `suave_runner` container, including tests, ROS launches, `colcon`, and direct `pytest` runs. The host machine is not assumed to have SUAVE or ROS dependencies installed. Use the container's default sourced workspace configuration; do not override `PYTHONPATH`, `ROS_LOG_DIR`, or similar ROS/Python environment variables unless the user explicitly asks. Exception: `suave_runner`'s `_run_launchfile` intentionally sets `ROS_LOG_DIR` per-run to redirect node logs into the result folder.

Default container command pattern:

```bash
docker exec suave_runner bash -lc 'cd /home/ubuntu-user/suave_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && <command>'
```

All build/test commands run from the **ROS workspace root inside the container** (`/home/ubuntu-user/suave_ws`), not from inside `src/suave`.

```bash
source /opt/ros/humble/setup.bash

# Install deps
rosdep install --from-paths src --ignore-src -r -y

# Build all
colcon build --symlink-install

# Low-memory build
colcon build --symlink-install --executor sequential --parallel-workers 1

# Build single package
colcon build --symlink-install --packages-select <package_name>

source install/setup.bash

# Test a package
colcon test --packages-select <package_name> --event-handlers console_direct+
colcon test-result --verbose

# Test all suave packages at once
colcon test --event-handlers console_cohesion+ --packages-select suave suave_bt suave_metacontrol suave_metrics suave_missions suave_monitor suave_msgs suave_none suave_random suave_runner suave_tools

# Auto-fix C++ style (run from inside the package directory)
ament_uncrustify --reformat

# Run Python tests directly (after sourcing)
python3 -m pytest -q <package>/test
```

Lint tests are pytest wrappers around `ament_flake8`, `ament_pep257`, and `ament_copyright`.

## Running SUAVE

Run SUAVE from inside the `suave_runner` container using the default workspace environment.

```bash
# Quick example
cd runner && ./example_run.sh

# ArduSub SITL (separate terminal)
sim_vehicle.py -L RATBeach -v ArduSub --model=JSON --console

# Simulation
ros2 launch suave simulation.launch.py x:=-17.0 y:=2.0

# Mission (default: no adaptation manager)
ros2 launch suave_missions mission.launch.py

# Mission with a specific manager
ros2 launch suave_missions mission.launch.py adaptation_manager:=bt result_filename:=measurement_1
# adaptation_manager values: none | metacontrol | random | bt

# Experiment runner (ROS2, config-file driven — preferred for campaigns)
ros2 launch suave_runner suave_runner.launch.py
# Config: suave_runner/config/runner_config.yml — controls experiments, disturbance timing, result_path

# Shell runner (simple positional args)
cd runner && ./runner.sh [true|false] [metacontrol|random|none|bt] [time|distance] <runs>
# headless_runner.sh is the same but uses screen instead of xfce4-terminal

# Results default to: ~/suave/results/
```

MAVROS default FCU URL: `udp://0.0.0.0:14550@14555` (avoids needing `sim_vehicle --out=...`).

## Docker

```bash
# Run GUI image
docker run -it --shm-size=512m -p 6901:6901 -e VNC_PW=password --security-opt seccomp=unconfined ghcr.io/kas-lab/suave:main

# Build all images locally
./build_docker_images.sh

# Build headless image (from repo root)
docker build -t suave-headless:dev -f docker/dockerfile-suave-headless .

# Syntax check
docker build --check -f docker/dockerfile-suave-headless .
docker build --check --build-arg BASE_IMAGE=kasm-jammy:dev -f docker/dockerfile-suave .
```

Dockerfiles are intentionally lowercase (`docker/dockerfile-*`). When checking `docker/dockerfile-suave` without a local base, GHCR may return `denied` — use `--build-arg BASE_IMAGE=kasm-jammy:dev` after building it locally.

**Image users and result paths:**
- GUI image (`suave:main`): user `kasm-user`, results at `/home/kasm-user/suave/results`
- Headless image (`suave-headless:main`): user `ubuntu-user`, results at `/home/ubuntu-user/suave/results`
- Headless image has no `CMD` — pass the runner command explicitly on `docker run`.

**Version pinning:** git SHAs live in `docker/versions.env`; Python package versions live in `requirements.txt` (repo root). `build_docker_images.sh` sources `versions.env` and forwards SHAs as `--build-arg`.

## Navigation / MAVROS Frame Convention

`suave/config/suave_mavros_apm_config.yaml` sets `local_position.frame_id: map`, so `mavros/local_position/pose` is published in the **Gazebo/world frame** — no conversion between Gazebo-sourced coordinates and MAVROS local positions is needed.
`MavrosPositionController.has_local_pose` is the readiness gate before sending any setpoint. The controller creates position ROS entities through its owning lifecycle node; it is not a separately spun node.
Spiral search (`spiral_search_lc.py`) initialises its center from the robot's MAVROS local position on activation — not from world origin.

## ROS Interfaces for Managing Subsystems

A compliant managing subsystem must interact with:
- **`/diagnostics`** — `diagnostic_msgs/DiagnosticArray`
- **`/task/request`** and **`/task/cancel`** — `suave_msgs/srv/Task`
- **system_modes** `ChangeMode` services: `/f_maintain_motion/change_mode`, `/f_generate_search_path/change_mode`, `/f_follow_pipeline/change_mode`

When adding a new managing subsystem: include SUAVE's base launch with `task_bridge` disabled and wire the new package into `suave_missions/launch/mission.launch.py` via an `adaptation_manager` condition.

## Runner / Metrics IPC
`mission_metrics/done` (`std_msgs/Bool`) signals run completion. Both the `MissionMetrics` publisher and `ExperimentRunnerNode` subscriber must declare `RELIABLE` reliability + `TRANSIENT_LOCAL` durability (depth 1). Mismatched QoS causes DDS to silently drop the connection.
Per-run logs are written to `<result_path>/logs/run_{exp_idx}_{run_idx}/`: `ardupilot.log` (raw SITL stdout/stderr), `simulation/` and `experiment/` (ROS node logs via `ROS_LOG_DIR`).
`resume_result_path` parameter resumes a crashed campaign from an existing result folder; `random_seed` (default `100`) controls perturbation reproducibility.

## Code Conventions

**Python (`ament_python` packages):**
- Python code must pass flake8 and pep257.
- Use **single-line docstrings** throughout — multi-line docstrings trigger D213.
- Nested `def` inside a function (common in launch files) needs a blank line before it (E306).
- Empty `__init__.py` files with only a copyright header must not have a trailing blank line (W391).
- Add the standard `Copyright 2026 KAS Lab` Apache-2.0 header to Python files:
  ```
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
- Nodes subclass `rclpy.node.Node`; declare/read ROS params in `__init__`; expose `main()` as console script entry point.
- `snake_case` for functions/methods/variables/file names; `PascalCase` for classes.
- No type hints unless they add clear value to surrounding context.
- `extras_require={'test': ['pytest']}` — do not use deprecated `tests_require`.
- PEP8/flake8-clean imports; pep257 docstrings on new public modules/classes/functions.

**Copyright tests:**
- New Python packages need `test/test_copyright.py` **without** `@pytest.mark.skip` (the skip is only a placeholder until headers are added).
- C++ packages: `set(ament_cmake_copyright_FOUND TRUE)` in CMakeLists disables the copyright check — remove it once all headers are in place.

**C++ (`suave_bt`, `suave_msgs`):**
- C++17, `-Wall -Wextra -Wpedantic`. Headers under `include/suave_bt`, implementations under `src/suave_bt`.
- BT node names registered in `src/suave_bt.cpp`; XML trees in `bts/`.

**Launch / Config:**
- Launch files are Python ROS launch descriptions installed via glob in `setup.py` or CMake.
- Keep MAVROS launch args (`fcu_url`, `gcs_url`, `mavros_config_yaml`, `mavros_pluginlists_yaml`) overrideable.
- After changing mission configs or package data files, `colcon build --symlink-install` may be needed.

**Licensing:** Apache-2.0 headers are present in many files; new substantive files should follow the nearby package header style.

## Documentation

`docs/source/` is a Sphinx GitHub Pages site that mirrors README content — keep both in sync when updating installation steps, Docker commands, or runner docs. Pages: `installation`, `docker`, `run`, `architecture`, `extend`, `implementations`, `metrics`, `troubleshooting`, `related`, `citing`, `api`.
When changing `suave_runner` behavior or parameters, update **both** `suave_runner/README.md` and `docs/source/run.md`.

The VCS dependencies file is `suave.repos` (vcs format). The old name `suave.rosinstall` is obsolete — do not reference it.

## Action Server Pattern (suave lifecycle nodes)

Action servers are registered in `on_configure()` so they remain discoverable. The `use_action_server` parameter defaults to `False`: legacy behavior starts from lifecycle activation when false, while action mode waits for a goal when true.

Use `make_goal_callback()`, `accept_cancel`, `use_action_server()`, and `lifecycle_state_is_active()` from `suave/suave/action_server_utils.py`; do not restore per-node callback copies or separate active flags.
Use `wait_for_action_completion()` before destroying resources used by an active execute callback during lifecycle cleanup or shutdown.
On ROS 2 Humble, `node._state_machine.current_state` is `(state_id, state_label)`, so the shared active-state helper checks element `1` for `active`. `_state_machine` is internal rclpy API, which is why access stays centralized.

**Threading events required per node:**
- `_abort_event = threading.Event()` in `__init__`; cleared in `on_activate`, set in `on_deactivate` and `on_shutdown`. Execute callbacks check it to call `abort()` on deactivation-interrupted goals.
- `_goal_executing = threading.Event()` in `__init__`; set at execute-callback entry, cleared in `finally`. `make_goal_callback` checks it via `getattr` for the single-flight guard — nodes without it get no guard.
- `on_cleanup` must call `self._action_server.destroy()` — omitting it leaks a second action server on the same name during configure→cleanup→configure cycles.
- `on_shutdown` must set `_abort_event` even if `on_deactivate` already does — `on_shutdown` can be called from `active` state, bypassing deactivation.

**Stop-predicate callables:** Pass `goal_handle.is_cancel_requested` as `lambda: goal_handle.is_cancel_requested`, not as a direct value. `is_cancel_requested` is a property (returns bool), not a method — passing it directly captures a one-time snapshot.

**Stop-reason enums:** When a node uses a `_StopReason`-style enum, add a `DEACTIVATED` variant. Map it to `goal_handle.abort()` in the execute callback so deactivation is distinguished from client-requested cancellation.

Keep action callbacks as policy wrappers around node-local core behavior. Core routines return explicit outcomes; action wrappers map them to `succeed()`, `abort()`, or `canceled()`, while lifecycle wrappers own task start and stop. Action mode must not leave a legacy timer or worker running concurrently. Long-running navigation must check its injected stop policy inside every setpoint wait, reached wait, and retry loop. Keep an in-progress path waypoint queued until it is reached so timeout, cancellation, or lifecycle deactivation can resume safely.

Use `call_service_with_timeout()` from `suave/suave/ros_service_utils.py` for bounded service waits. It uses `call_async()` and then waits for completion; do not describe or name it as a synchronous ROS call. On Humble, `Node.create_rate()` returns `rclpy.timer.Rate`; there is no importable `rclpy.rate.Rate`.

**Testing action servers:** Use `spin_nodes_in_executors()` and the future/result helpers from `suave/test/action_test_utils.py`. The helper node and node under test need separate executors because re-entrant `spin_until_future_complete()` can otherwise fail with `ValueError: generator already executing`. Cover terminal-state mapping and cancellation/timeout inside inner waits, not only goal acceptance. `rclpy.shutdown() has been called` can appear on stderr after successful package tests; only treat it as known noise when pytest and colcon report success.
Mock `GoalHandle` objects in unit tests need `is_cancel_requested = False` when the execute callback wraps it in a lambda stop predicate. Monkeypatched core methods that accept `cancel_requested=None` must be patched as `lambda cancel_requested=None: <value>` — a plain `lambda: <value>` will fail when the kwarg is passed.

**New action types:** Put definitions in `suave_msgs/action/`; add `find_package(action_msgs REQUIRED)` and `DEPENDENCIES action_msgs` in `suave_msgs/CMakeLists.txt`, plus `<depend>action_msgs</depend>` in `suave_msgs/package.xml`. Python action support comes from the ROS package `rclpy`; `rclpy_action` is not a valid rosdep dependency.

**Name collision:** If a class name matches an action type, alias the action import, for example `from suave_msgs.action import RechargeBattery as RechargeBatteryAction`.

## Before Finishing Changes

- `git status --short` — distinguish own edits from pre-existing uncommitted experiment/Docker changes; do not revert unrelated work.
- SUAVE validation must run inside `suave_runner` with the container's default sourced workspace environment; do not run ROS/SUAVE tests on the host.
- Python changes: `colcon test --packages-select <pkg> --event-handlers console_direct+`.
- C++/message changes: build then test.
- Launch file changes: `python3 -m py_compile <launch_file.py>`.
- Shell script changes: `bash -n <script>`.
- Dockerfile changes: `docker build --check`.
- If `setup.py` data_files or console script entry points change, verify they are installed correctly after build.
- Note any tests skipped because ROS/Gazebo/ArduSub/Docker dependencies are unavailable in the environment.
