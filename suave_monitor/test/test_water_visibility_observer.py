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

import math
from unittest.mock import Mock

from builtin_interfaces.msg import Time
from mavros_msgs.msg import State
import pytest
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from suave_monitor.water_visibility_observer import WaterVisibilityObserver


@pytest.fixture
def observer(monkeypatch):
    rclpy.init()
    publisher = Mock()
    create_publisher = Node.create_publisher

    def make_publisher(node, msg_type, topic, *args, **kwargs):
        if topic == '/diagnostics':
            return publisher
        return create_publisher(node, msg_type, topic, *args, **kwargs)

    monkeypatch.setattr(Node, 'create_publisher', make_publisher)
    node = WaterVisibilityObserver()
    clock = Mock()
    clock.now.return_value.to_msg.return_value = Time(sec=100)
    monkeypatch.setattr(node, 'get_clock', lambda: clock)
    try:
        yield node, publisher, clock
    finally:
        node.destroy_node()
        rclpy.shutdown()


def visibility(publisher):
    return float(publisher.publish.call_args.args[0].status[0].values[0].value)


def set_time(clock, seconds):
    clock.now.return_value.to_msg.return_value = Time(sec=seconds)


def test_publishes_on_construction_and_keeps_timer_active(observer):
    node, publisher, _ = observer
    publisher.publish.assert_called_once()
    assert visibility(publisher) == pytest.approx(3.75)
    assert not node.qa_publisher_timer.is_canceled()


@pytest.mark.parametrize('startup_delay', [0, 37, 240])
def test_visibility_schedule_starts_at_guided(observer, startup_delay):
    node, publisher, clock = observer
    node.set_parameters([
        Parameter('water_visibility_period', value=120),
        Parameter('water_visibility_sec_shift', value=15.0),
    ])
    initial_value = 2.5 + 1.25 * math.cos(2 * math.pi * 15 / 120)

    node.status_cb(State(mode='STABILIZE'))
    set_time(clock, 100 + startup_delay)
    node.qa_publisher_cb()
    assert visibility(publisher) == pytest.approx(initial_value)

    timer = node.qa_publisher_timer
    node.status_cb(State(mode='GUIDED'))
    node.qa_publisher_cb()
    assert visibility(publisher) == pytest.approx(initial_value)
    assert node.qa_publisher_timer is timer

    set_time(clock, 130 + startup_delay)
    node.status_cb(State(mode='STABILIZE'))
    node.status_cb(State(mode='GUIDED'))
    node.qa_publisher_cb()
    expected = 2.5 + 1.25 * math.cos(2 * math.pi * 45 / 120)
    assert visibility(publisher) == pytest.approx(expected)


def test_experiment_one_visibility_stays_constant(observer):
    node, publisher, clock = observer
    node.set_parameters([
        Parameter('water_visibility_min', value=2.5),
        Parameter('water_visibility_max', value=2.5),
    ])
    node.qa_publisher_cb()
    assert visibility(publisher) == 2.5
    node.status_cb(State(mode='GUIDED'))
    set_time(clock, 350)
    node.qa_publisher_cb()
    assert visibility(publisher) == 2.5
