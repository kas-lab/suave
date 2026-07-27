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

"""Helpers for ROS action-server tests."""

from contextlib import contextmanager
import threading
import time

from rclpy.executors import MultiThreadedExecutor


def wait_for_future(future, timeout_sec, poll_sec=0.05):
    """Return True when the future completes before the timeout."""
    return wait_for_condition(future.done, timeout_sec, poll_sec=poll_sec)


def wait_for_condition(predicate, timeout_sec, poll_sec=0.05):
    """Return True when predicate becomes true before the timeout."""
    deadline = time.time() + timeout_sec
    while not predicate() and time.time() < deadline:
        time.sleep(poll_sec)
    return predicate()


def send_goal_and_wait(client, goal, timeout_sec=5.0, feedback_callback=None):
    """Send an action goal and return its goal handle, or None on timeout."""
    future = client.send_goal_async(goal, feedback_callback=feedback_callback)
    if not wait_for_future(future, timeout_sec):
        return None
    return future.result()


def wait_for_action_result(goal_handle, timeout_sec=10.0, poll_sec=0.1):
    """Return an action result response, or None on timeout."""
    future = goal_handle.get_result_async()
    if not wait_for_future(future, timeout_sec, poll_sec=poll_sec):
        return None
    return future.result()


@contextmanager
def spin_nodes_in_executors(*nodes):
    """Spin each node in its own executor for the duration of a test."""
    executors = []
    threads = []
    for node in nodes:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        executors.append(executor)
        threads.append(thread)
    try:
        yield executors
    finally:
        for executor in executors:
            executor.shutdown()
        for thread in threads:
            thread.join(timeout=2.0)
