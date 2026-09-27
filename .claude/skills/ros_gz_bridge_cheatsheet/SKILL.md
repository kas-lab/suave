---
name: ros-gz-bridge-cheatsheet
description: Use when adding, reviewing, or debugging ros_gz_bridge parameter_bridge mappings between ROS 2 and Gazebo / Ignition topics.
---

# ros_gz_bridge Mapping Cheat Sheet

Use this skill when editing `ros_gz_bridge` / `parameter_bridge` nodes in launch files.

## Bridge argument syntax

`parameter_bridge` topic arguments have this shape:

```text
/topic@ROS_TYPE@GZ_TYPE   # bidirectional
/topic@ROS_TYPE[GZ_TYPE   # Gazebo -> ROS
/topic@ROS_TYPE]GZ_TYPE   # ROS -> Gazebo
```

Direction symbols:

- `@`: bidirectional bridge
- `[`: Gazebo Transport to ROS 2
- `]`: ROS 2 to Gazebo Transport

Examples:

```python
# Gazebo pose topic into ROS
arguments=['/model/bluerov2/pose@geometry_msgs/msg/Pose[gz.msgs.Pose']

# ROS current command into Gazebo
arguments=['/ocean_current@geometry_msgs/msg/Point]gz.msgs.Vector3d']

# Bidirectional string topic
arguments=['/chatter@std_msgs/msg/String@gz.msgs.StringMsg']
```

Note: Older help text and examples may say `ignition.msgs.*`; this workspace uses `gz.msgs.*` in launch files.

## Common SUAVE mappings

| ROS 2 type | Gazebo type | Notes |
| --- | --- | --- |
| `geometry_msgs/msg/Point` | `gz.msgs.Vector3d` | 3D vector / point values, e.g. ocean current |
| `geometry_msgs/msg/Vector3` | `gz.msgs.Vector3d` | 3D vector values |
| `geometry_msgs/msg/Pose` | `gz.msgs.Pose` | Single pose |
| `geometry_msgs/msg/PoseArray` | `gz.msgs.Pose_V` | Pose array / repeated poses |
| `geometry_msgs/msg/Twist` | `gz.msgs.Twist` | Linear and angular velocity |
| `std_msgs/msg/Bool` | `gz.msgs.Boolean` | Boolean values |
| `std_msgs/msg/Float64` | `gz.msgs.Double` | Scalar double |
| `std_msgs/msg/String` | `gz.msgs.StringMsg` | Strings |
| `sensor_msgs/msg/Image` | `gz.msgs.Image` | Images |
| `sensor_msgs/msg/CameraInfo` | `gz.msgs.CameraInfo` | Camera calibration/info |
| `sensor_msgs/msg/Imu` | `gz.msgs.IMU` | IMU readings |
| `sensor_msgs/msg/LaserScan` | `gz.msgs.LaserScan` | 2D lidar |
| `sensor_msgs/msg/PointCloud2` | `gz.msgs.PointCloudPacked` | Point cloud data |
| `nav_msgs/msg/Odometry` | `gz.msgs.Odometry` | Odometry |
| `ros_gz_interfaces/msg/Entity` | `gz.msgs.Entity` | Gazebo entity identifiers |

Always verify unusual mappings against the installed `ros_gz_bridge` version.

## Where bridges belong in SUAVE

- Gazebo-specific bridge nodes usually belong in `src/suave/suave/launch/simulation.launch.py`.
- Managed-system ROS nodes usually belong in `src/suave/suave/launch/suave.launch.py`.
- If a bridge should always be active for the simulator topic, do not put it behind an optional managed-node condition unless requested.

## Parameter bridge launch pattern

```python
gz_ocean_current_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/ocean_current@geometry_msgs/msg/Point]gz.msgs.Vector3d'],
    output=print_output,
    name='gz_ocean_current_bridge',
)
```

For long topic/type strings, split adjacent Python strings to satisfy flake8:

```python
arguments=[
    '/model/min_pipes_pipeline/pose@geometry_msgs/msg/PoseArray@'
    'gz.msgs.Pose_V'],
```

## Verify bridge syntax in the active container

```bash
docker exec suave bash -lc 'cd /home/ubuntu-user/suave_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run ros_gz_bridge parameter_bridge --help'
```

The command may print help and exit nonzero; the help text is still useful for confirming syntax.

## Debugging checklist

- Confirm direction: use `]` when ROS publishes and Gazebo consumes; use `[` when Gazebo publishes and ROS consumes.
- Confirm exact ROS topic name, including leading slash.
- Confirm Gazebo topic exists using Gazebo CLI tools in the container if available.
- Confirm ROS topic exists with `ros2 topic list` and type with `ros2 topic info`.
- Confirm the bridge process is running and did not exit due to an invalid mapping.
- If a launch argument controls the ROS node, decide whether the bridge should also be conditional or always active.
