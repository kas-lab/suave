# Suggested Commands

Important SUAVE execution rule:
- Run SUAVE builds, tests, ROS commands, and runtime checks inside a development container. Standalone containers are commonly named `suave` or `suave_runner`; `suave_rebetmc_ws` normally uses `suave_rebetmc`. Inspect container mounts if the context is ambiguous.
- Do not assume the host has SUAVE or ROS dependencies installed. Do not run SUAVE tests or ROS commands on the host.
- Use the container's default sourced workspace configuration; do not override `PYTHONPATH`, `ROS_LOG_DIR`, or similar ROS/Python environment variables unless explicitly requested.
- Container wrapper: `docker exec <container-name> bash -lc 'cd /home/ubuntu-user/suave_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && <command>'`.

Workspace setup/build inside container:
- Build all packages: `docker exec <container-name> bash -lc 'cd /home/ubuntu-user/suave_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && colcon build --symlink-install'`.
- Low-memory build: `docker exec <container-name> bash -lc 'cd /home/ubuntu-user/suave_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && colcon build --symlink-install --executor sequential --parallel-workers 1'`.
- Build selected package: `docker exec <container-name> bash -lc 'cd /home/ubuntu-user/suave_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && colcon build --symlink-install --packages-select <package_name>'`.

Testing/linting inside container:
- Test selected package: `docker exec <container-name> bash -lc 'cd /home/ubuntu-user/suave_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test --packages-select <package_name> --event-handlers console_direct+'`.
- Show test results: `docker exec <container-name> bash -lc 'cd /home/ubuntu-user/suave_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test-result --verbose'`.
- Run focused Python tests: `docker exec <container-name> bash -lc 'cd /home/ubuntu-user/suave_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && python3 -m pytest -q -rs src/suave/<package>/test/<test_file>.py'`.
- Common package lint tests are pytest tests wrapping `ament_flake8`, `ament_pep257`, and `ament_copyright`.

Running SUAVE inside container:
- Quick example: `docker exec <container-name> bash -lc 'cd /home/ubuntu-user/suave_ws/src/suave/runner && source /opt/ros/humble/setup.bash && source /home/ubuntu-user/suave_ws/install/setup.bash && ./example_run.sh'`.
- Start ArduSub SITL: `docker exec <container-name> bash -lc 'cd /home/ubuntu-user/suave_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && sim_vehicle.py -L RATBeach -v ArduSub --model=JSON --console'`.
- Start simulation: `docker exec <container-name> bash -lc 'cd /home/ubuntu-user/suave_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch suave simulation.launch.py x:=-17.0 y:=2.0'`.
- Start default mission: `docker exec <container-name> bash -lc 'cd /home/ubuntu-user/suave_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch suave_missions mission.launch.py'`.
- Start mission with manager: `docker exec <container-name> bash -lc 'cd /home/ubuntu-user/suave_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch suave_missions mission.launch.py adaptation_manager:=bt result_filename:=measurement_1'`.
- Runner launch: `docker exec <container-name> bash -lc 'cd /home/ubuntu-user/suave_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch suave_runner suave_runner.launch.py'`.

Docker/container inspection:
- List containers: `docker ps --format '{{.Names}}\t{{.Status}}\t{{.Image}}'`.
- Inspect `suave_runner` mounts: `docker inspect suave_runner --format '{{json .Mounts}}'`.

Useful Linux/repo commands on host:
- Fast file search: `rg --files`.
- Fast text search: `rg '<pattern>'`.
- Git status: `git status --short`.
- Inspect package metadata: `find . -maxdepth 3 -name package.xml -print`.