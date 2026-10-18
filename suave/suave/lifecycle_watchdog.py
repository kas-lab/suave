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

"""Report lifecycle activations requested by mode_manager that never happen."""

import time

from lifecycle_msgs.msg import State
from lifecycle_msgs.msg import TransitionEvent

from rcl_interfaces.msg import ParameterDescriptor

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

# The `type: node` parts of config/suave_modes.yaml.
WATCHED_NODES = [
    'f_generate_search_path_node',
    'f_follow_pipeline_node',
    'f_maintain_motion_node',
    'generate_recharge_path_node',
]


class LifecycleWatchdog(Node):
    """Publish mission/control_failure when a requested activation is lost."""

    # system_modes' mode_manager sends ACTIVATE without waiting for the
    # result, so a lost transition goes unnoticed. This node only reports it:
    # it never calls change_state or retries, which would bias results.

    def __init__(self, node_name='lifecycle_watchdog', **kwargs):
        """Subscribe to the request and transition events of each node."""
        super().__init__(node_name, **kwargs)
        self.declare_parameter(
            'watched_nodes', WATCHED_NODES, ParameterDescriptor(
                description='Lifecycle nodes managed by mode_manager'))
        self.declare_parameter(
            'activation_timeout', 10.0, ParameterDescriptor(
                description='Seconds a node has to become active after '
                            "mode_manager's activate request"))
        self.activation_timeout = self.get_parameter(
            'activation_timeout').value
        self._deadlines = {}
        self._active = set()
        self._failure_pub = self.create_publisher(
            String, 'mission/control_failure', 10)
        for node in self.get_parameter('watched_nodes').value:
            self.create_subscription(
                TransitionEvent, f'/{node}/transition_request_info',
                lambda msg, node=node: self._request_cb(node, msg), 10)
            self.create_subscription(
                TransitionEvent, f'/{node}/transition_event',
                lambda msg, node=node: self._transition_cb(node, msg), 10)
        self.create_timer(0.1, self._check_deadlines)

    def _request_cb(self, node, msg):
        # A request may arrive after the transition event it caused, since
        # they use different topics; an already active node needs no deadline.
        if (msg.goal_state.id == State.PRIMARY_STATE_ACTIVE and
                node not in self._active):
            self._deadlines[node] = time.monotonic() + self.activation_timeout
        else:
            self._deadlines.pop(node, None)

    def _transition_cb(self, node, msg):
        if msg.goal_state.id == State.PRIMARY_STATE_ACTIVE:
            self._active.add(node)
            self._deadlines.pop(node, None)
        else:
            self._active.discard(node)

    def _check_deadlines(self):
        now = time.monotonic()
        for node, deadline in list(self._deadlines.items()):
            if now < deadline:
                continue
            del self._deadlines[node]
            reason = (f'{node} did not become active within '
                      f"{self.activation_timeout}s of mode_manager's activate "
                      'request')
            self.get_logger().error(reason)
            self._failure_pub.publish(String(data=reason))


def main(args=None):
    """Run the lifecycle watchdog."""
    rclpy.init(args=args)
    node = LifecycleWatchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
