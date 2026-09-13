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

from datetime import datetime
import importlib.util
import json
import os
from pathlib import Path
import sys

import numpy as np

import pandas as pd

import pytest

import yaml


SOURCE = Path(__file__).parents[1] / 'suave_runner' / 'analysis'
SPEC = importlib.util.spec_from_file_location(
    'wilcoxon_analysis', SOURCE / 'wilcoxon_analysis.py')
engine = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(engine)
sys.modules['wilcoxon_analysis'] = engine
SORT_SPEC = importlib.util.spec_from_file_location(
    'sort_results', SOURCE / 'sort_results.py')
sorter = importlib.util.module_from_spec(SORT_SPEC)
SORT_SPEC.loader.exec_module(sorter)
sys.modules['sort_results'] = sorter
SPEC = importlib.util.spec_from_file_location(
    'wilcoxon_analysis_batch', SOURCE / 'wilcoxon_analysis_batch.py')
batch = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(batch)


def _frame(offset=0):
    return pd.DataFrame({
        'run_idx': [0, 1, 2, 3, 4],
        'pipeline found': [True, True, True, True, False],
        'time searching pipeline (s)': np.array([10, 20, 30, 40, 50]) + offset,
        'distance inspected (m)': np.array([0, 5, 10, 15, 20]) + offset,
    })


def _write_raw(campaign, method, exp_idx, frame):
    raw = frame.copy()
    dates = []
    for position, run_idx in enumerate(raw['run_idx']):
        timestamp = 1700000000 + exp_idx * 10000 + position * 300
        dates.append(datetime.fromtimestamp(timestamp).strftime(
            '%d-%b-%Y-%H-%M-%S'))
        marker = campaign / f'run_{exp_idx}_{run_idx}.done'
        marker.touch()
        os.utime(marker, (timestamp + 5, timestamp + 5))
    raw['datetime'] = dates
    raw.drop(columns='run_idx').to_csv(
        campaign / f'method{method}_suave.csv', index=False)


def _campaign(root, name='exp1', count=2):
    campaign = root / 'campaigns' / name
    folder = campaign / 'sorted'
    folder.mkdir(parents=True)
    experiments = []
    # Experiment indices deliberately differ from alphabetical method order.
    for exp_idx, method in enumerate(reversed(range(count))):
        frame = _frame(method + 1)
        if method == 1:
            frame = frame.iloc[::-1]
        _write_raw(campaign, method, exp_idx, frame)
        frame.to_csv(folder / f'method{method}_suave_sorted.csv', index=False)
        experiments.append(json.dumps({
            'adaptation_manager': f'method{method}',
            'mission_name': 'suave', 'num_runs': 5,
        }))
    config = campaign / f'{name}_runner_config.yml'
    config.write_text(yaml.safe_dump({
        '/suave_runner_node': {
            'ros__parameters': {'experiments': experiments}},
    }))
    state_file = root / 'state.json'
    state = json.loads(state_file.read_text()) if state_file.exists() else {
        'campaigns': {}}
    state['campaigns'][f'{name}_suave_runner.launch.py'] = {
        'config_file': str(config), 'result_path': str(campaign)}
    state_file.write_text(json.dumps(state))
    return campaign


@pytest.mark.parametrize('column, values, message', [
    ('run_idx', [0, 0, 2, 3, 4], 'duplicate'),
    ('run_idx', [0, 1, None, 3, 4], 'non-finite'),
    ('run_idx', [0, 1, 2.5, 3, 4], 'nonnegative integers'),
    ('time searching pipeline (s)', [1, 2, 'bad', 4, 5], 'non-numeric'),
    ('distance inspected (m)', [1, 2, np.inf, 4, 5], 'non-finite'),
    ('distance inspected (m)', [1, 2, None, 4, 5], 'non-finite'),
    ('pipeline found', ['true', 'false', 'unknown', 'true', 'true'],
     'true/false'),
])
def test_reject_invalid_data(tmp_path, column, values, message):
    """Reject invalid data before invoking the analysis engine."""
    frame = _frame()
    frame[column] = values
    path = tmp_path / 'input.csv'
    frame.to_csv(path, index=False)
    with pytest.raises(ValueError, match=message):
        batch.validate_csv(path)


def test_reject_missing_status(tmp_path):
    """Do not allow missing success status to bypass detection filtering."""
    path = tmp_path / 'input.csv'
    _frame().drop(columns='pipeline found').to_csv(path, index=False)
    with pytest.raises(ValueError, match='missing required columns'):
        batch.validate_csv(path)


def test_reject_unmatched_keys_before_output(tmp_path):
    """Do not silently inner-join incomplete campaign pairs."""
    campaign = _campaign(tmp_path)
    path = campaign / 'sorted' / 'method1_suave_sorted.csv'
    _frame().iloc[:-1].to_csv(path, index=False)
    with pytest.raises(ValueError, match='run_idx mismatch'):
        batch.campaign_manifest(campaign)
    assert not (tmp_path / 'output').exists()


def test_batch_output_pairing_filtering_and_correction(tmp_path):
    """Reuse directional tests and apply Holm separately within campaigns."""
    _campaign(tmp_path, 'exp1', 3)
    _campaign(tmp_path, 'exp2', 2)
    plot = tmp_path / 'campaings_results' / 'exp1' / 'q-q-plots' / 'keep.png'
    plot.parent.mkdir(parents=True)
    plot.write_bytes(b'existing plot')
    assert batch.main([str(tmp_path)]) == 0
    for campaign, expected_rows in [('exp1', 12), ('exp2', 4)]:
        folder = (tmp_path / 'campaings_results' / campaign /
                  'wilcoxon_analysis')
        results = pd.read_csv(folder / f'{campaign}_wilcoxon_results.csv')
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
            assert (folder / f'{campaign}_wilcoxon_{suffix}.csv').is_file()
    assert plot.read_bytes() == b'existing plot'


def test_all_zero_differences_are_reported(tmp_path):
    """Keep zero inspected distance when both systems found the pipeline."""
    campaign = _campaign(tmp_path)
    frame = _frame()
    frame['distance inspected (m)'] = 0
    for exp_idx, method in enumerate([1, 0]):
        _write_raw(campaign, method, exp_idx, frame)
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


def test_bad_campaign_does_not_stop_other_campaigns(tmp_path, capsys):
    """Continue independent campaigns while returning a failure exit code."""
    (tmp_path / 'campaigns' / 'bad').mkdir(parents=True)
    _campaign(tmp_path, 'good')
    assert batch.main([str(tmp_path)]) == 1
    assert 'Error: bad:' in capsys.readouterr().err
    result = (tmp_path / 'campaings_results' / 'good' / 'wilcoxon_analysis' /
              'good_wilcoxon_results.csv')
    assert result.is_file()


def test_missing_batch(tmp_path, capsys):
    """Explain that the requested input needs a campaigns directory."""
    assert batch.main([str(tmp_path / 'missing')]) == 1
    assert 'Campaigns directory does not exist' in capsys.readouterr().err


def test_sorting_pre_step_recovers_ids_and_preserves_inputs(tmp_path):
    """Sort retried runs from markers without requiring sorted CSVs."""
    campaign = _campaign(tmp_path)
    for path in (campaign / 'sorted').iterdir():
        path.unlink()
    (campaign / 'sorted').rmdir()
    before = {path: (path.read_bytes(), path.stat().st_mtime_ns)
              for path in campaign.iterdir() if path.is_file()}
    assert batch.main([str(tmp_path)]) == 0
    for method in (0, 1):
        frame = pd.read_csv(
            campaign / 'sorted' / f'method{method}_suave_sorted.csv')
        assert frame['run_idx'].tolist() == [0, 1, 2, 3, 4]
        assert frame['time searching pipeline (s)'].tolist() == [
            10 + method + 1, 20 + method + 1, 30 + method + 1,
            40 + method + 1, 50 + method + 1]
    after = {p: (p.read_bytes(), p.stat().st_mtime_ns) for p in before}
    assert before == after
    sorted_paths = list((campaign / 'sorted').glob('*.csv'))
    mtimes = {p: p.stat().st_mtime_ns for p in sorted_paths}
    batch.sort_campaign(campaign)
    assert mtimes == {p: p.stat().st_mtime_ns for p in sorted_paths}


@pytest.mark.parametrize('failure', ['missing_marker', 'timestamp_gap'])
def test_sort_failure_blocks_analysis_and_preserves_sorted(tmp_path, failure):
    """Keep existing outputs intact when reconstruction cannot be trusted."""
    campaign = _campaign(tmp_path)
    before = {p: p.read_bytes() for p in (campaign / 'sorted').glob('*.csv')}
    marker = campaign / 'run_0_0.done'
    if failure == 'missing_marker':
        marker.unlink()
    else:
        timestamp = marker.stat().st_mtime + 1000
        os.utime(marker, (timestamp, timestamp))
    assert batch.main([str(tmp_path)]) == 1
    assert before == {p: p.read_bytes() for p in before}
    assert not (tmp_path / 'campaings_results').exists()


def test_config_dir_handles_moved_config_paths(tmp_path):
    """Use explicit original configs when the recorded absolute path moved."""
    campaign = _campaign(tmp_path)
    state_file = tmp_path / 'state.json'
    state = json.loads(state_file.read_text())
    state['campaigns']['exp1_suave_runner.launch.py']['config_file'] = (
        '/missing/install/config/exp1_runner_config.yml')
    state_file.write_text(json.dumps(state))
    assert batch.main([str(tmp_path)]) == 1
    assert batch.main([str(tmp_path), '--config-dir', str(campaign)]) == 0


def test_sorting_rejects_incomplete_campaign(tmp_path):
    """Require the run count declared in the campaign configuration."""
    campaign = _campaign(tmp_path)
    config = campaign / 'exp1_runner_config.yml'
    document = yaml.safe_load(config.read_text())
    values = document['/suave_runner_node']['ros__parameters']['experiments']
    entry = json.loads(values[0])
    entry['num_runs'] = 6
    values[0] = json.dumps(entry)
    config.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match='config declares 6'):
        batch.sort_campaign(campaign)


def test_stale_sorted_methods_are_not_analyzed(tmp_path):
    """Only include methods declared in the current campaign configuration."""
    campaign = _campaign(tmp_path)
    extra = campaign / 'sorted' / 'unrelated_sorted.csv'
    extra.write_text('invalid stale data')
    assert batch.main([str(tmp_path)]) == 0
    result = pd.read_csv(tmp_path / 'campaings_results' / 'exp1' /
                         'wilcoxon_analysis' / 'exp1_wilcoxon_results.csv')
    assert len(result) == 4
    assert extra.read_text() == 'invalid stale data'


def test_new_state_schema_and_relative_config(tmp_path):
    """Resolve newer campaign-name keys and relative config paths."""
    campaign = _campaign(tmp_path)
    config = campaign / 'exp1_runner_config.yml'
    (tmp_path / 'state.json').write_text(json.dumps({
        'schema_version': 2,
        'campaigns': {'exp1': {
            'config_file': str(config.relative_to(tmp_path)),
        }},
    }))
    assert batch.resolve_config(campaign) == config
