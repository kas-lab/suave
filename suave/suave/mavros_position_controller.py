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

"""MAVROS local-position state and setpoint publishing helpers."""

from typing import Optional

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


class MavrosPositionController:
    """Manage MAVROS local-position state for an owning ROS node."""

    def __init__(
            self, node: Node, ground_depth_gz: float,
            callback_group: CallbackGroup) -> None:
        """Create MAVROS position entities through the owning node."""
        self._node = node
        self._ground_depth_gz = ground_depth_gz
        self._local_pose: Optional[PoseStamped] = None

        local_position_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=5,
        )
        self._local_position_subscription = node.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._local_pose_callback,
            local_position_qos,
            callback_group=callback_group,
        )
        self._setpoint_publisher = node.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10,
        )

    @property
    def local_pose(self) -> Optional[PoseStamped]:
        """Return the latest MAVROS local pose, if one was received."""
        return self._local_pose

    @property
    def has_local_pose(self) -> bool:
        """Return whether MAVROS has published a local pose."""
        return self._local_pose is not None

    def _local_pose_callback(self, pose: PoseStamped) -> None:
        """Store the latest MAVROS local pose."""
        self._local_pose = pose

    def publish_gazebo_setpoint(
            self, pose: Pose,
            altitude: float) -> Optional[PoseStamped]:
        """Publish a Gazebo XY pose at the requested ground altitude."""
        return self.publish_xy_setpoint(
            pose.position.x,
            pose.position.y,
            altitude,
        )

    def publish_xy_setpoint(
            self, x: float, y: float,
            altitude: float) -> Optional[PoseStamped]:
        """Publish an XY setpoint at the requested ground altitude."""
        if not self.has_local_pose or self._setpoint_publisher is None:
            return None

        setpoint = PoseStamped()
        setpoint.header.stamp = self._node.get_clock().now().to_msg()
        setpoint.pose.position.x = x
        setpoint.pose.position.y = y
        setpoint.pose.position.z = self._ground_depth_gz + altitude
        setpoint.pose.orientation.w = 1.0
        self._setpoint_publisher.publish(setpoint)
        return setpoint

    def is_setpoint_reached(
            self, setpoint: PoseStamped, tolerance: float) -> bool:
        """Return whether the current pose is within XYZ tolerance."""
        local_pose = self._local_pose
        if local_pose is None:
            return False
        return (
            abs(local_pose.pose.position.x -
                setpoint.pose.position.x) <= tolerance
            and abs(local_pose.pose.position.y -
                    setpoint.pose.position.y) <= tolerance
            and abs(local_pose.pose.position.z -
                    setpoint.pose.position.z) <= tolerance
        )

    def is_xy_setpoint_reached(
            self, setpoint: PoseStamped, tolerance: float) -> bool:
        """Return whether the current pose is within XY tolerance."""
        local_pose = self._local_pose
        if local_pose is None:
            return False
        return (
            abs(local_pose.pose.position.x -
                setpoint.pose.position.x) <= tolerance
            and abs(local_pose.pose.position.y -
                    setpoint.pose.position.y) <= tolerance
        )

    def destroy(self) -> None:
        """Destroy controller-owned ROS entities."""
        if self._local_position_subscription is not None:
            self._node.destroy_subscription(
                self._local_position_subscription)
            self._local_position_subscription = None
        if self._setpoint_publisher is not None:
            self._node.destroy_publisher(self._setpoint_publisher)
            self._setpoint_publisher = None
