# Copyright 2026 KAS-lab
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

"""Test recovery diagnostics updating the monitor's aggregate state."""

from unittest.mock import Mock

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import pytest

import rclpy
from rclpy.node import Node

from suave_monitor.thruster_monitor import ThrusterMonitor


@pytest.fixture
def monitor(monkeypatch):
    """Create a monitor with captured diagnostics and no MAVROS calls."""
    rclpy.init()
    publisher = Mock()
    create_publisher = Node.create_publisher

    def make_publisher(node, msg_type, topic, *args, **kwargs):
        if topic == '/diagnostics':
            return publisher
        return create_publisher(node, msg_type, topic, *args, **kwargs)

    monkeypatch.setattr(Node, 'create_publisher', make_publisher)
    node = ThrusterMonitor()
    node.call_service = Mock()
    node.thrusters_operational['1'] = False
    node.thrusters_operational['2'] = False
    try:
        yield node, publisher
    finally:
        node.destroy_node()
        rclpy.shutdown()


def recovery(*thrusters):
    """Match the recovery node's component diagnostic format."""
    return DiagnosticArray(status=[DiagnosticStatus(
        level=DiagnosticStatus.OK, name='', message='Component status',
        values=[KeyValue(key=f'c_thruster_{number}', value='RECOVERED')
                for number in thrusters])])


def published_values(publisher):
    """Extract aggregate diagnostic values."""
    message = publisher.publish.call_args.args[0]
    return {value.key: value.value for value in message.status[0].values}


def test_recovery_updates_only_reported_thruster(monitor):
    """A confirmed recovery clears only the corresponding failed entry."""
    node, publisher = monitor
    node.recovery_diagnostics_cb(recovery(1))
    assert node.thrusters_operational['1'] is True
    assert node.thrusters_operational['2'] is False
    assert published_values(publisher) == {
        'operational_thrusters': '5', 'operational_thrusters_delta': '1'}
    node.recovery_diagnostics_cb(recovery(2))
    assert published_values(publisher) == {
        'operational_thrusters': '6', 'operational_thrusters_delta': '0'}
    node.call_service.assert_not_called()


def test_duplicates_and_own_diagnostics_do_not_loop(monitor):
    """Repeated notifications and aggregate messages cause no republishing."""
    node, publisher = monitor
    node.recovery_diagnostics_cb(recovery(1, 1, 2))
    publisher.publish.assert_called_once()
    aggregate = publisher.publish.call_args.args[0]
    node.recovery_diagnostics_cb(aggregate)
    node.recovery_diagnostics_cb(recovery(1, 2))
    publisher.publish.assert_called_once()


@pytest.mark.parametrize('field,value', [
    ('key', 'c_thruster_99'), ('key', 'c_thruster_invalid'),
    ('key', 'other_1'), ('value', 'FALSE'), ('value', 'OK'),
    ('message', 'QA status'), ('level', DiagnosticStatus.ERROR),
])
def test_unrelated_or_invalid_diagnostics_are_ignored(monitor, field, value):
    """Only known thrusters with successful recovery messages are updated."""
    node, publisher = monitor
    message = recovery(1)
    target = (message.status[0].values[0] if field in ('key', 'value')
              else message.status[0])
    setattr(target, field, value)
    node.recovery_diagnostics_cb(message)
    assert node.thrusters_operational['1'] is False
    assert len(node.thrusters_operational) == 6
    publisher.publish.assert_not_called()


def test_scheduled_failure_can_happen_after_recovery(monitor):
    """Recovery does not disable future failure events."""
    node, publisher = monitor
    node.recovery_diagnostics_cb(recovery(1, 2))
    node.change_thruster_status('1', 'failure')
    node.publish_operational_thrusters()
    assert published_values(publisher)['operational_thrusters_delta'] == '1'
