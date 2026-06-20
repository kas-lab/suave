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

"""Tests for mission metrics parameters."""

from pathlib import Path

import pytest

import rclpy
from rclpy.parameter import Parameter

from suave_metrics.mission_metrics import MissionMetrics

import yaml


@pytest.fixture
def rclpy_context():
    """Create an isolated rclpy context."""
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        yield context
    finally:
        rclpy.shutdown(context=context)


def _mission_metrics(context, parameter_overrides):
    return MissionMetrics(
        'test_mission_metrics',
        context=context,
        parameter_overrides=parameter_overrides)


def test_correct_water_visibility_threshold_parameter_is_used(rclpy_context):
    """Verify the correctly spelled threshold parameter is used."""
    thresholds = [4.0, 3.0, 2.0]
    node = _mission_metrics(
        rclpy_context,
        [Parameter('water_visibility_threshold', value=thresholds)])
    try:
        assert node.get_water_visibility_thresholds() == thresholds
    finally:
        node.destroy_node()


def test_legacy_water_visibility_threshold_parameter_still_works(
        rclpy_context):
    """Verify the old misspelled parameter remains a fallback."""
    thresholds = [4.5, 3.5, 2.5]
    node = _mission_metrics(
        rclpy_context,
        [Parameter('water_visibiity_threshold', value=thresholds)])
    try:
        assert node.get_water_visibility_thresholds() == thresholds
    finally:
        node.destroy_node()


def test_correct_water_visibility_threshold_overrides_legacy(
        rclpy_context):
    """Verify the correctly spelled parameter wins when both are set."""
    thresholds = [4.0, 3.0, 2.0]
    legacy_thresholds = [8.0, 7.0, 6.0]
    node = _mission_metrics(
        rclpy_context,
        [
            Parameter('water_visibility_threshold', value=thresholds),
            Parameter('water_visibiity_threshold', value=legacy_thresholds),
        ])
    try:
        assert node.get_water_visibility_thresholds() == thresholds
    finally:
        node.destroy_node()


@pytest.mark.parametrize(
    'config_path',
    sorted((Path(__file__).parents[2] / 'suave_missions' / 'config').glob(
        '*mission_config.yaml'))
)
def test_mission_configs_use_correct_water_visibility_parameter(
        config_path):
    """Verify mission YAML files use the correctly spelled parameter."""
    config = yaml.safe_load(config_path.read_text())
    params = config['/mission_metrics']['ros__parameters']

    assert 'water_visibility_threshold' in params
    assert 'water_visibiity_threshold' not in params
