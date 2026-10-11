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

"""Test requirement-fulfillment plotting helpers."""

import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest


def load_plotter():
    """Load the plotting script as a module."""
    script = Path(__file__).parents[1] / 'scripts' / (
        'plot_requirement_fulfillment.py')
    spec = importlib.util.spec_from_file_location('plotter', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_step_resample_uses_bag_origin_and_previous_value():
    """Verify late samples are not shifted to zero percent."""
    plotter = load_plotter()
    grid = np.asarray([0.0, 20.0, 60.0, 70.0, 100.0])
    values = plotter.step_resample(
        [2_000_000_000, 7_000_000_000], [0.2, 0.8],
        10_000_000_000, grid)
    np.testing.assert_allclose(values, [np.nan, 0.2, 0.2, 0.8, 0.8])


def test_duplicate_timestamps_keep_last_value():
    """Verify duplicate timestamp samples use the last recorded value."""
    plotter = load_plotter()
    values = plotter.step_resample([2, 2], [0.2, 0.4], 10,
                                   np.asarray([10.0, 20.0, 30.0]))
    np.testing.assert_allclose(values, [np.nan, 0.4, 0.4])


def test_mean_and_ci_ignore_missing_values():
    """Verify aggregation uses finite samples only."""
    plotter = load_plotter()
    mean, ci, counts = plotter.mean_and_ci([
        np.asarray([np.nan, 0.5, 1.0]),
        np.asarray([np.nan, 1.0, 0.0]),
    ])
    assert np.isnan(mean[0])
    np.testing.assert_allclose(mean[1:], [0.75, 0.5])
    assert ci[0] == 0.0 and np.all(ci[1:] > 0.0)
    np.testing.assert_array_equal(counts, [0, 2, 2])


def test_failed_and_pending_bags_are_never_selected(tmp_path):
    """Verify quarantined bags with data are excluded or rejected."""
    plotter = load_plotter()
    for name in ('accepted', 'failed/quarantined', 'pending/unfinished'):
        bag = tmp_path / 'rosbags' / name
        bag.mkdir(parents=True)
        (bag / 'metadata.yaml').write_text('fixture')
    (tmp_path / 'run_0_0.done').touch()
    status = tmp_path / 'run_0_0.status.json'

    def write_status(bag_path):
        status.write_text(json.dumps({
            'schema_version': 1, 'run_id': 'run_0_0', 'attempt_id': 'a1',
            'adaptation_manager': 'bt', 'status': 'completed',
            'success': True, 'execution_status': 'completed',
            'finalization_state': 'finished', 'recording_enabled': True,
            'log_dir': 'logs/run_0_0/a1', 'attempts': [],
            'started_epoch': 0.0, 'finished_epoch': 1.0,
            'bag': {'state': 'accepted', 'path': bag_path,
                    'accepted_path': bag_path},
        }))

    write_status('rosbags/accepted')
    _, selected, excluded = plotter.accepted_entries(
        tmp_path / 'rosbags', ['bt'])
    assert [entry['bag_path'] for entry in selected] == ['rosbags/accepted']
    assert {item['bag_path'] for item in excluded} == {
        'rosbags/failed/quarantined', 'rosbags/pending/unfinished'}
    for quarantined in ('rosbags/failed/quarantined',
                        'rosbags/pending/unfinished'):
        write_status(quarantined)
        with pytest.raises(ValueError, match='accepted root'):
            plotter.accepted_entries(tmp_path / 'rosbags', ['bt'])


def test_plot_no_data_and_requirement_outputs(tmp_path):
    """Verify output generation works for data and no-data cases."""
    plotter = load_plotter()
    grid = np.linspace(0.0, 100.0, 3)
    report = {'per_grid_counts': {}}
    plotter.plot_requirement(
        'thruster_availability',
        plotter.REQUIREMENTS['thruster_availability'],
        {'rebetmc': [np.asarray([np.nan, 0.5, 0.8])], 'random': []},
        ['rebetmc', 'random', 'metacontrol'], grid, tmp_path, ['png'], report,
        {'rebetmc': 1, 'random': 1})
    assert report['no_observations'] == ['Random']
    assert report['no_accepted_runs'] == ['Metacontrol']
    plotter.plot_no_data_requirement(
        'battery_reaction_time',
        plotter.REQUIREMENTS['battery_reaction_time'],
        tmp_path, ['png'])
    assert (tmp_path / 'thruster_availability.png').is_file()
    assert (tmp_path / 'battery_reaction_time.png').is_file()
    assert report['per_grid_counts']['rebetmc'] == [0, 1, 1]
    assert plotter.MANAGER_LABELS['rebetmc'] == 'TacticalRetreat'
