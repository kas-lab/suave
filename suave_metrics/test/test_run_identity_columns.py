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

"""Tests for the run_idx and attempt_id columns of every metrics CSV."""

import csv
from pathlib import Path

import pytest
import rclpy
from rclpy.parameter import Parameter

from suave_metrics.mission_metrics import MissionMetrics

CSV_SUFFIXES = ('', '_component_recovery_time', '_wv_reaction_time',
                '_battery_reaction_time')


@pytest.fixture
def rclpy_context():
    """Create an isolated rclpy context."""
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        yield context
    finally:
        rclpy.shutdown(context=context)


def _save_results(context, result_path):
    node = MissionMetrics(
        'test_mission_metrics_run_identity', context=context,
        parameter_overrides=[
            Parameter('result_path', value=str(result_path)),
            Parameter('result_filename', value='bt_suave')])
    try:
        node.mission_start_time = node.get_clock().now()
        node.component_recovery_time = [1.0]
        node.wv_reaction_time = [2.0]
        node.battery_reaction_time = [3.0]
        node.save_mission_results()
    finally:
        Path('/tmp/mission.done').unlink(missing_ok=True)
        node.destroy_node()
    rows = {}
    for suffix in CSV_SUFFIXES:
        with open(result_path / f'bt_suave{suffix}.csv', newline='') as handle:
            rows[suffix] = list(csv.reader(handle))
    return rows


def test_csvs_record_runner_identity(rclpy_context, tmp_path, monkeypatch):
    monkeypatch.setenv('SUAVE_RUN_IDX', '7')
    monkeypatch.setenv('SUAVE_ATTEMPT_ID', 'abc123')
    for suffix, (header, row) in _save_results(
            rclpy_context, tmp_path).items():
        assert header[-2:] == ['run_idx', 'attempt_id'], suffix
        assert row[-2:] == ['7', 'abc123'], suffix
        assert len(row) == len(header), suffix


def test_csvs_leave_identity_empty_outside_runner(
        rclpy_context, tmp_path, monkeypatch):
    monkeypatch.delenv('SUAVE_RUN_IDX', raising=False)
    monkeypatch.delenv('SUAVE_ATTEMPT_ID', raising=False)
    for suffix, (header, row) in _save_results(
            rclpy_context, tmp_path).items():
        assert header[-2:] == ['run_idx', 'attempt_id'], suffix
        assert row[-2:] == ['', ''], suffix
