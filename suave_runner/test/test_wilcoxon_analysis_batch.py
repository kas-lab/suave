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

"""Test Wilcoxon batch validation, pairing, correction, and output layout."""

import importlib.util
import json
from pathlib import Path
import sys

import numpy as np

import pandas as pd

import pytest

import rclpy
from rclpy.parameter import Parameter


SOURCE = Path(__file__).parents[1] / 'suave_runner' / 'analysis'


def _load(name):
    spec = importlib.util.spec_from_file_location(name, SOURCE / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


common = _load('common')
engine = _load('wilcoxon_analysis')
batch = _load('wilcoxon_analysis_batch')


def _frame(offset=0):
    return pd.DataFrame({
        'run_idx': [0, 1, 2, 3, 4],
        'pipeline found': [True, True, True, True, False],
        'time searching pipeline (s)': np.array([10, 20, 30, 40, 50]) + offset,
        'distance inspected (m)': np.array([0, 5, 10, 15, 20]) + offset,
    })


@pytest.fixture
def campaign(tmp_path, metrics_campaign):
    """Build batch campaigns whose method0 run 2 was retried once."""
    def build(name='exp1', count=2, frames=None):
        # Experiment indices deliberately differ from alphabetical order.
        frames = frames or {
            f'method{method}': _frame(method + 1).iloc[::-1 if method else 1]
            for method in reversed(range(count))}
        return metrics_campaign(tmp_path / 'campaigns' / name, frames,
                                retried={('method0', 2)})

    return build


@pytest.mark.parametrize('column, values, message', [
    ('run_idx', [0, 0, 2, 3, 4], 'duplicate'),
    ('run_idx', [0, 1, None, 3, 4], 'non-finite'),
    ('run_idx', [0, 1, 2.5, 3, 4], 'nonnegative integers'),
    ('time searching pipeline (s)', [1, 2, 'bad', 4, 5], 'non-numeric'),
    ('distance inspected (m)', [1, 2, np.inf, 4, 5], 'non-finite'),
    ('distance inspected (m)', [1, 2, None, 4, 5], 'non-finite'),
    ('pipeline found', ['true', 'false', 'unknown', 'true', 'true'],
     'invalid pipeline found'),
])
def test_reject_invalid_data(column, values, message):
    """Reject invalid data before invoking the analysis engine."""
    frame = _frame()
    frame[column] = values
    with pytest.raises(ValueError, match=message):
        batch.validate_frame(frame, 'input')


def test_reject_missing_status():
    """Do not allow missing success status to bypass detection filtering."""
    with pytest.raises(ValueError, match='missing required columns'):
        batch.validate_frame(_frame().drop(columns='pipeline found'), 'input')


def test_reject_unmatched_keys_before_output(tmp_path, campaign):
    """Do not silently inner-join incomplete campaign pairs."""
    path = campaign(frames={'method1': _frame(), 'method0': _frame()[:-1]})
    with pytest.raises(ValueError, match='run_idx mismatch'):
        batch.campaign_systems(path)
    assert not (tmp_path / 'campaign_results').exists()


def test_load_system_keeps_only_accepted_attempt(campaign):
    """Drop the failed attempt's row of a retried run."""
    path = campaign() / 'method0_suave.csv'
    assert len(pd.read_csv(path)) == 6
    frame = engine.load_system('method0', path)
    assert frame['run_idx'].tolist() == [0, 1, 2, 3, 4]
    assert frame['time searching pipeline (s)'].tolist() == [
        11, 21, 31, 41, 51]
    assert frame['pipeline found'].tolist() == [True] * 4 + [False]


def test_load_system_requires_runs_json_and_identity(tmp_path, campaign):
    """Refuse CSVs that cannot be matched to accepted attempts."""
    path = campaign() / 'method0_suave.csv'
    pd.read_csv(path).drop(columns='attempt_id').to_csv(path, index=False)
    with pytest.raises(ValueError, match='attempt_id'):
        engine.load_system('method0', path)
    loose = tmp_path / 'loose.csv'
    _frame().to_csv(loose, index=False)
    with pytest.raises(ValueError, match='no runs.json'):
        engine.load_system('method0', loose)
    with pytest.raises(ValueError, match='completed managers'):
        engine.load_system('unknown', path.with_name('method1_suave.csv'))


def test_wilcoxon_node_analyzes_accepted_attempts(tmp_path, campaign):
    """The ROS node loads configured raw CSVs through the same filter."""
    path = campaign()
    rclpy.init()
    try:
        node = engine.WilcoxonAnalysis(parameter_overrides=[
            Parameter('result_path', value=str(tmp_path / 'node')),
            Parameter('data_files', value=[json.dumps({
                'managing_system': method,
                'data_file': str(path / f'{method}_suave.csv')})
                for method in ('method0', 'method1')]),
        ])
        try:
            node.perform_analysis()
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()
    results = pd.read_csv(
        tmp_path / 'node' / 'suave_wilcoxon_analysis_results.csv')
    assert set(results['declared_pairs']) == {5}
    assert set(results['n_used']) == {4}


def test_batch_output_pairing_filtering_and_correction(tmp_path, campaign):
    """Reuse directional tests and apply Holm separately within campaigns."""
    campaign('exp1', 3)
    campaign('exp2', 2)
    plot = tmp_path / 'campaign_results' / 'exp1' / 'q-q-plots' / 'keep.png'
    plot.parent.mkdir(parents=True)
    plot.write_bytes(b'existing plot')
    assert batch.main([str(tmp_path), '--correction', 'holm']) == 0
    for name, expected_rows in [('exp1', 12), ('exp2', 4)]:
        folder = (tmp_path / 'campaign_results' / name /
                  'wilcoxon_analysis')
        results = pd.read_csv(folder / f'{name}_wilcoxon_results.csv')
        assert len(results) == expected_rows
        assert set(results['declared_pairs']) == {5}
        assert set(results['n_used']) == {4}
        assert set(results['excluded_extra'].astype(str)) == {'4'}
        assert set(results['correction']) == {'holm'}
        np.testing.assert_allclose(results['p_adjusted'],
                                   engine.holm_correction(results['p_raw']))
        row = results[(results.row == 'method0') &
                      (results.col == 'method1') &
                      (results.metric == 'search_time')].iloc[0]
        assert row['alternative'] == 'less'
        assert row['median_diff'] == -1
        assert row['n_neg'] == 4
        assert row['effect_size'] == -1
        for suffix in ('time_search_pipeline', 'distance_inspected'):
            assert (folder / f'{name}_wilcoxon_{suffix}.csv').is_file()
    assert plot.read_bytes() == b'existing plot'


def test_all_zero_differences_are_reported(tmp_path, campaign):
    """Keep zero inspected distance when both systems found the pipeline."""
    frame = _frame()
    frame['distance inspected (m)'] = 0
    campaign(frames={'method1': frame, 'method0': frame})
    output = tmp_path / 'custom'
    assert batch.main([str(tmp_path), '--output', str(output),
                       '--correction', 'none']) == 0
    results = pd.read_csv(output / 'exp1' / 'wilcoxon_analysis' /
                          'exp1_wilcoxon_results.csv')
    assert results['p_raw'].isna().all()
    assert results['p_adjusted'].isna().all()
    assert set(results['n_used']) == {4}
    assert set(results['n_zero']) == {4}
    assert results['note'].str.contains('test not computed').all()


def test_bad_campaign_does_not_stop_other_campaigns(
        tmp_path, campaign, capsys):
    """Continue independent campaigns while returning a failure exit code."""
    campaign('bad', frames={'method0': _frame()})
    campaign('good')
    assert batch.main([str(tmp_path)]) == 1
    assert 'Error: bad:' in capsys.readouterr().err
    result = (tmp_path / 'campaign_results' / 'good' / 'wilcoxon_analysis' /
              'good_wilcoxon_results.csv')
    assert result.is_file()


def test_missing_batch(tmp_path, capsys):
    """Explain that the requested input does not exist."""
    assert batch.main([str(tmp_path / 'missing')]) == 1
    assert 'not found' in capsys.readouterr().err


@pytest.mark.parametrize('relative', ['', 'campaigns', 'campaigns/exp1'])
def test_accepts_batch_campaigns_dir_and_campaign(
        tmp_path, campaign, relative):
    """Every input form writes under the batch's campaign_results."""
    campaign()
    assert batch.main([str(tmp_path / relative)]) == 0
    assert (tmp_path / 'campaign_results' / 'exp1' / 'wilcoxon_analysis' /
            'exp1_wilcoxon_results.csv').is_file()
