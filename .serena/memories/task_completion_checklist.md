# Task Completion Checklist

Before finishing code changes:
- Check `git status --short` and distinguish own edits from pre-existing user edits. This repository is often used with uncommitted experiment/Docker changes, so avoid reverting unrelated changes.
- SUAVE validation must run inside the applicable development or integration container. Standalone containers are commonly named `suave` or `suave_runner`; `suave_rebetmc_ws` normally uses `suave_rebetmc`. Do not assume the host has ROS/SUAVE dependencies.
- Use the container's default sourced workspace configuration. Do not override `PYTHONPATH`, `ROS_LOG_DIR`, or similar ROS/Python environment variables unless explicitly requested.
- Validation wrapper: `docker exec <container-name> bash -lc 'cd /home/ubuntu-user/suave_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && <command>'`.
- Python files should include the standard `Copyright 2026 KAS Lab` Apache-2.0 header.
- For Python package changes, run inside the container: `colcon test --packages-select <package_name> --event-handlers console_direct+`. For narrow focused tests, direct `python3 -m pytest -q -rs src/suave/<package>/test/<test_file>.py` is acceptable inside the same sourced container workspace.
- For C++/message package changes, run inside the container: `colcon build --symlink-install --packages-select <package_name>` and then `colcon test --packages-select <package_name> --event-handlers console_direct+`.
- After `colcon test`, run `colcon test-result --verbose` inside the container if failures occur or to summarize results.
- If mission config, launch, or package data files change, verify they are included in the relevant `setup.py` `data_files` or CMake `install()` rules.
- If adding/changing console scripts, verify `setup.py` entry points and package dependencies in `package.xml`.
- For launch changes, at minimum run `python3 -m py_compile <launch_file.py>` inside the container; for YAML config changes, parse the touched YAML files inside the container if available.
- For Dockerfile or Docker script changes, run `docker build --check` on the affected Dockerfile when possible. For `docker/dockerfile-suave`, use `--build-arg BASE_IMAGE=kasm-jammy:dev` if GHCR cannot authorize the default base. When updating pinned versions: git SHAs go in `docker/versions.env`, Python package versions go in `requirements.txt` (repo root).
- When updating README installation steps, Docker commands, or runner documentation, also apply the same changes to the matching page in `docs/source/` (Sphinx GitHub Pages site). The two must stay in sync.
- For shell script changes, run `bash -n <script>` inside the container when the script belongs to SUAVE runtime/build workflows.
- If touching MAVROS simulation wiring, preserve the working default `fcu_url` of `udp://0.0.0.0:14550@14555` unless intentionally changing the `sim_vehicle` port setup.
- If adding a new managing subsystem, verify ROS interface compatibility with `/diagnostics`, `/task/request`, `/task/cancel`, and the system_modes change mode services.
- Note any tests that could not be run because the applicable container is unavailable or missing required Gazebo/ArduSub/MAVROS/Docker dependencies.