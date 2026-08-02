# SUAVE Container Execution Rule

Run SUAVE builds, tests, ROS commands, and runtime checks inside a development container. Standalone SUAVE containers are commonly named `suave` or `suave_runner`; integration workspaces may use another name. In `suave_rebetmc_ws`, normally use `suave_rebetmc` because SUAVE is mounted there. If the context is ambiguous, inspect running containers and mounts before choosing.

Do not assume the host machine has SUAVE or ROS dependencies installed. Do not run SUAVE tests or ROS commands on the host.

Use the container's default sourced workspace configuration and avoid overriding `PYTHONPATH`, `ROS_LOG_DIR`, or similar ROS/Python environment variables unless the user explicitly asks.

Command pattern (replace the placeholder with the applicable container name):

```bash
docker exec <container-name> bash -lc 'cd /home/ubuntu-user/suave_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && <command>'
```

Example focused pytest:

```bash
docker exec <container-name> bash -lc 'cd /home/ubuntu-user/suave_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && python3 -m pytest -q -rs src/suave/suave/test/test_recover_thrusters_lc.py'
```