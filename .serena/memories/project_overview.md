# Project Overview

SUAVE is the Self-adaptive Underwater Autonomous Vehicle Exemplar. It models a single AUV performing a pipeline inspection mission: searching for a pipeline, following it, and inspecting it. The repo separates a managed subsystem (vehicle mission functionality) from managing subsystems (adaptation logic), so different adaptation managers can be connected through ROS 2 interfaces.

Primary domain/runtime stack:
- ROS 2 Humble workspace with mostly Python `ament_python` packages and some C++ `ament_cmake` packages.
- Gazebo Harmonic simulation, ArduSub/ArduPilot SITL, MAVROS/mavros_wrapper, ros_gz.
- BehaviorTree.CPP for the `suave_bt` managing subsystem.
- MROS2/Metacontrol dependencies for `suave_metacontrol`.
- Docker/Kasm packaging for browser-accessible images plus a headless runner image.

Key ROS interfaces for managing subsystems:
- `/diagnostics` topic using `diagnostic_msgs/DiagnosticArray`.
- `/task/request` and `/task/cancel` services using `suave_msgs/srv/Task`.
- system_modes `ChangeMode` services for lifecycle modes: `/f_maintain_motion/change_mode`, `/f_generate_search_path/change_mode`, `/f_follow_pipeline/change_mode`.

Important entrypoints:
- ArduSub SITL: `sim_vehicle.py -L RATBeach -v ArduSub --model=JSON --console`.
- Simulation: `ros2 launch suave simulation.launch.py x:=-17.0 y:=2.0`.
- MAVROS simulation default FCU URL is `udp://0.0.0.0:14550@14555`; this avoids needing `sim_vehicle --out=127.0.0.1:14551`.
- Mission: `ros2 launch suave_missions mission.launch.py` with launch args such as `adaptation_manager:=none|metacontrol|random|bt` and `result_filename:=...`.
- Runner: `ros2 launch suave_runner suave_runner.launch.py` or `ros2 run suave_runner suave_runner --ros-args ...`.
- Result analysis: `ros2 run suave_runner statistical_analysis --ros-args ...` for pairwise inference; `ros2 run suave_runner summarize_results <campaign-folder>` for descriptive statistics and optional LaTeX output.
- Example runner script: `cd runner && ./example_run.sh`.

Always check `git status --short` before edits; this repository is often used with local uncommitted experiment/Docker changes.