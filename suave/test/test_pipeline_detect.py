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

"""Tests for the pipeline detection node."""

import math

from diagnostic_msgs.msg import DiagnosticStatus

from geometry_msgs.msg import Pose

import pytest

import rclpy

from suave.pipeline_detection import PipelineDetection


class FakePublisher:
    """Capture published messages for assertions."""

    def __init__(self):
        """Initialize the published message list."""
        self.messages = []

    def publish(self, msg):
        """Store a published message."""
        self.messages.append(msg)


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
def pipeline_node():
    """Create the pipeline detection node under test."""
    node = PipelineDetection()
    try:
        yield node
    finally:
        node.destroy_node()


def make_pose(x=0.0, y=0.0, z=0.0):
    """Create a pose for tests."""
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    return pose


def test_camera_fov_parameter_defaults_to_sixty_degrees(pipeline_node):
    """Verify the camera FOV parameter default is converted to radians."""
    camera_fov_degrees = pipeline_node.get_parameter('camera_fov').value
    assert camera_fov_degrees == pytest.approx(60.0)
    assert pipeline_node.camera_fov == pytest.approx(math.pi / 3)


def test_get_coverage_area_uses_camera_fov_and_altitude(pipeline_node):
    """Verify coverage area is computed from altitude and FOV."""
    bluerov_pose = make_pose(z=-1.0)
    pipe_pose = make_pose(z=-4.0)

    coverage_area = pipeline_node.get_coverage_area(bluerov_pose, pipe_pose)

    delta = 3.0 * math.tan((math.pi / 3) / 2)
    assert coverage_area == pytest.approx((2 * delta) * (2 * delta))


def test_publish_coverage_area_publishes_diagnostic(pipeline_node):
    """Verify coverage area diagnostics use the expected schema."""
    fake_publisher = FakePublisher()
    pipeline_node.diagnostics_publisher = fake_publisher

    pipeline_node.publish_coverage_area(12.5)

    assert len(fake_publisher.messages) == 1
    diagnostic = fake_publisher.messages[0]
    assert len(diagnostic.status) == 1
    status = diagnostic.status[0]
    assert status.level == DiagnosticStatus.OK
    assert status.name == 'pipeline_detection: Camera coverage area'
    assert status.message == 'QA status'
    assert len(status.values) == 1
    assert status.values[0].key == 'coverage_area'
    assert status.values[0].value == '12.5'


def test_detect_pipeline_publishes_current_coverage_area(pipeline_node):
    """Verify pose callbacks publish coverage before detection results."""
    published_coverage = []
    bluerov_pose = make_pose(z=-1.0)
    pipe_pose = make_pose(x=100.0, y=100.0, z=-4.0)
    pipeline_node.interpolated_path.poses.append(pipe_pose)

    pipeline_node.publish_coverage_area = published_coverage.append

    pipeline_node.detect_pipeline_cb(bluerov_pose)

    assert len(published_coverage) == 1
    delta = 3.0 * math.tan((math.pi / 3) / 2)
    assert published_coverage[0] == pytest.approx((2 * delta) * (2 * delta))
