# Codebase Structure

Top-level folders/files:
- `README.md`: comprehensive user/developer documentation (install, run, Docker, extend, troubleshoot). Keep in sync with `docs/source/`.
- `docs/source/`: Sphinx GitHub Pages site mirroring README content. Pages: `installation`, `docker`, `run`, `architecture`, `extend`, `implementations`, `metrics`, `troubleshooting`, `related`, `citing`, `api`. Changes to installation steps, Docker commands, or runner docs must be reflected in both.
- `suave.repos`: VCS dependency file used by CI, workspace setup, and Docker image builds for external deps. The old name `suave.rosinstall` is obsolete — do not reference it.
- `docker/versions.env`: shell-sourceable file with pinned git SHAs (`ARDUSUB_COMMIT`, `ARDUPILOT_GAZEBO_COMMIT`). Sourced by `build_docker_images.sh` and forwarded as `--build-arg` to both Dockerfiles.
- `requirements.txt` (repo root): pinned Python package versions for all images and local installs. Both Dockerfiles `COPY` and `pip install -r` this file.
- `docker/`: Docker/Kasm image definitions and install scripts. Important files include `docker/dockerfile-kasm-core-jammy`, `docker/dockerfile-suave`, `docker/dockerfile-suave-headless`, and `docker/src/ubuntu/install/kasm_vnc/install_kasm_vnc.sh`.
- `runner/`: shell scripts for running the exemplar and experiments.
- `.github/workflows/`: CI; `main.yml` uses `ros-tooling/action-ros-ci` targeting ROS 2 Humble, and `container.yml` builds/pushes the browser and headless images.

ROS packages:
- `suave/` (`ament_python`): managed subsystem functionality, launch files, and simulation config. `suave/config/suave_mavros_apm_config.yaml` and `suave/config/suave_mavros_apm_pluginlists.yaml` tailor MAVROS for the lightweight simulation.
- `suave_monitor/` (`ament_python`): monitor nodes such as `thruster_monitor`, `battery_monitor`, `water_visibility_observer` that publish diagnostics/status.
- `suave_missions/` (`ament_python`): mission planners and launch files. Console scripts include `const_dist_mission` and `time_constrained_mission`.
- `suave_metrics/` (`ament_python`): metrics collection, console script `mission_metrics`.
- `suave_runner/` (`ament_python`): experiment orchestration and result analysis. Console scripts: `suave_runner`, `statistical_analysis`, and `summarize_results`; contains focused pytest tests. Its Dockerfile lives at `docker/dockerfile-suave-headless` so it can copy the local checkout.
- `suave_tools/` (`ament_python`): auxiliary tools/config such as PlotJuggler config.
- `suave_msgs/` (`ament_cmake`): ROS service definitions `Task.srv` and `GetPath.srv` via `rosidl_generate_interfaces`.
- `suave_managing/suave_none/` (`ament_python`): no-manager variant launch package.
- `suave_managing/suave_random/` (`ament_python`): random managing subsystem, console script `task_bridge_random`.
- `suave_managing/suave_metacontrol/` (`ament_python`): MROS2/metacontrol managing subsystem, console script `task_bridge_metacontrol`.
- `suave_managing/suave_bt/` (`ament_cmake`): C++ BehaviorTree.CPP managing subsystem. Sources under `src/suave_bt`, headers under `include/suave_bt`, BT XML files under `bts/`, launch files under `launch/`.

Configuration:
- Mission configs live in `suave_missions/config/*.yaml`.
- Runner config lives in `suave_runner/config/runner_config.yml` — controls experiments list, disturbance timing, result_path, GUI flag.
- Package data is installed through each package's `setup.py` or `CMakeLists.txt`.

Known external managing subsystem implementations:
- `suave_rosa` (https://github.com/kas-lab/rosa): knowledge-based, uses TypeDB; paper in Frontiers in Robotics and AI (2025).
- `suave_planta` (https://github.com/kas-lab/suave_planta): PDDL-based, TOMASys ontology → PDDL translation, PlanSys2 for planning/execution.