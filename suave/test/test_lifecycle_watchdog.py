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

"""Tests for the report-only lifecycle watchdog."""

import os
import time

from action_test_utils import spin_nodes_in_executors
from action_test_utils import wait_for_condition

from lifecycle_msgs.msg import State
from lifecycle_msgs.msg import TransitionEvent

import pytest

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import String

from suave.lifecycle_watchdog import LifecycleWatchdog
from suave.lifecycle_watchdog import WATCHED_NODES

import yaml

WATCHED = 'watched_test_node'
TIMEOUT = 0.5


@pytest.fixture(scope='session', autouse=True)
def rclpy_runtime():
    """Initialize rclpy for the test module."""
    if not rclpy.ok():
        rclpy.init()
    try:
        yield
    finally:
        if rclpy.ok():
            rclpy.shutdown()


@pytest.fixture
def watchdog_and_helper():
    """Spin a watchdog and a helper node in separate executors."""
    watchdog = LifecycleWatchdog(parameter_overrides=[
        Parameter('watched_nodes', value=[WATCHED]),
        Parameter('activation_timeout', value=TIMEOUT),
    ])
    helper = Node('watchdog_test_helper')
    failures = []
    helper.create_subscription(
        String, 'mission/control_failure',
        lambda msg: failures.append(msg.data), 10)
    request_pub = helper.create_publisher(
        TransitionEvent, f'/{WATCHED}/transition_request_info', 1)
    event_pub = helper.create_publisher(
        TransitionEvent, f'/{WATCHED}/transition_event', 10)
    with spin_nodes_in_executors(watchdog, helper):
        assert wait_for_condition(
            lambda: (request_pub.get_subscription_count() > 0 and
                     event_pub.get_subscription_count() > 0 and
                     helper.count_publishers('mission/control_failure') > 0),
            5.0)
        yield request_pub, event_pub, failures
    watchdog.destroy_node()
    helper.destroy_node()


def _event(goal_state):
    msg = TransitionEvent()
    msg.goal_state.id = goal_state
    return msg


def test_lost_activation_publishes_one_failure(watchdog_and_helper):
    """Report an activate request that no transition follows, once."""
    request_pub, _event_pub, failures = watchdog_and_helper
    request_pub.publish(_event(State.PRIMARY_STATE_ACTIVE))
    assert wait_for_condition(lambda: failures, 5.0)
    time.sleep(4 * TIMEOUT)
    assert len(failures) == 1
    assert failures[0].startswith(f'{WATCHED} did not become active within')


def test_completed_activation_publishes_nothing(watchdog_and_helper):
    """Stay silent when the node reaches active after the request."""
    request_pub, event_pub, failures = watchdog_and_helper
    request_pub.publish(_event(State.PRIMARY_STATE_ACTIVE))
    event_pub.publish(_event(State.TRANSITION_STATE_ACTIVATING))
    event_pub.publish(_event(State.PRIMARY_STATE_ACTIVE))
    time.sleep(4 * TIMEOUT)
    assert failures == []


def test_default_watched_nodes_match_mode_config():
    """Watch every `type: node` part of suave_modes.yaml."""
    path = os.path.join(os.path.dirname(__file__), '..', 'config',
                        'suave_modes.yaml')
    with open(path) as config:
        parts = yaml.safe_load(config)
    nodes = [name for name, params in parts.items()
             if params['ros__parameters']['type'] == 'node']
    assert sorted(WATCHED_NODES) == sorted(nodes)
