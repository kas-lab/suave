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

"""Shared helpers for lifecycle-node action servers."""

from functools import wraps

import rclpy
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse


def spin_srv(executor):
    """Spin the executor, ignoring shutdown-related errors."""
    try:
        executor.spin()
    except (rclpy.executors.ExternalShutdownException, ValueError):
        pass


def lifecycle_state_is_active(node):
    """Return True when the lifecycle node is in the active state."""
    return node._state_machine.current_state[1] == 'active'


def when_lifecycle_active(func):
    """Decorate a method to run only when its lifecycle node is active."""
    @wraps(func)
    def inner(self, *args, **kwargs):
        if lifecycle_state_is_active(self):
            return func(self, *args, **kwargs)
    return inner


def use_action_server(node):
    """Return the use_action_server parameter value."""
    return node.get_parameter(
        'use_action_server').get_parameter_value().bool_value


def make_goal_callback(node):
    """Return a goal callback gated by lifecycle state, action mode, and single-flight."""
    def goal_callback(goal_request):
        if not lifecycle_state_is_active(node):
            node.get_logger().warn('Goal rejected: node is not active.')
            return GoalResponse.REJECT
        if not use_action_server(node):
            node.get_logger().warn(
                'Goal rejected: use_action_server is False.')
            return GoalResponse.REJECT
        executing = getattr(node, '_goal_executing', None)
        if executing is not None and executing.is_set():
            node.get_logger().warn('Goal rejected: goal already executing.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT
    return goal_callback


def accept_cancel(goal_handle):
    """Accept action cancellation."""
    return CancelResponse.ACCEPT
