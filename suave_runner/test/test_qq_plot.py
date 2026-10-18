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

"""Check standalone plotting, paired alignment, and input error handling."""

import importlib.util
from pathlib import Path
import sys

import matplotlib

import numpy as np

import pandas as pd

import pytest


matplotlib.use('Agg')
SCRIPT = (Path(__file__).parents[1] / 'suave_runner' /
          'analysis' / 'qq_plot.py')
COMMON_SPEC = importlib.util.spec_from_file_location(
    'common', SCRIPT.with_name('common.py'))
common = importlib.util.module_from_spec(COMMON_SPEC)
sys.modules['common'] = common
COMMON_SPEC.loader.exec_module(common)
SPEC = importlib.util.spec_from_file_location('qq_plot', SCRIPT)
qq_plot = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(qq_plot)
sys.modules['qq_plot'] = qq_plot
BATCH_SPEC = importlib.util.spec_from_file_location(
    'qq_plot_batch', SCRIPT.with_name('qq_plot_batch.py'))
qq_plot_batch = importlib.util.module_from_spec(BATCH_SPEC)
BATCH_SPEC.loader.exec_module(qq_plot_batch)


def _runs(values):
    return pd.DataFrame({'run_idx': range(len(values)), 'metric': values})


@pytest.fixture
def campaign(metrics_campaign):
    """Build a campaign with three methods; a's run 1 was retried once."""
    def build(path, methods=('a', 'b', 'c')):
        return metrics_campaign(
            path, {method: _runs([1, 3, 5]) for method in methods},
            retried={('a', 1)})

    return build


def test_single_csv_preserves_pairing_and_sign():
    """Remove missing pairs jointly and keep the A minus B direction."""
    frame = pd.DataFrame({'a': [9, np.nan, 5, 7, 12],
                          'b': [3, 4, 8, np.nan, 10]})
    difference, dropped = qq_plot.paired_differences(
        frame, 'a', 'input', column_b='b')
    np.testing.assert_array_equal(difference, [6, -3, 2])
    assert dropped == 2


def test_two_csvs_align_shuffled_run_ids():
    """Pair by identifier even when CSV rows have different orders."""
    a = pd.DataFrame({'run_idx': [0, 1, 2, 3], 'x': [10, 20, 30, 40]})
    b = pd.DataFrame({'run_idx': [2, 0, 3, 1], 'x': [25, 7, 31, np.nan]})
    difference, dropped = qq_plot.paired_differences(
        a, 'x', 'a.csv', b, 'x', 'b.csv')
    np.testing.assert_array_equal(difference, [3, 5, 9])
    assert dropped == 1


@pytest.mark.parametrize('ids, message', [
    ([0, 0, 2], 'unique, nonmissing'),
    ([0, None, 2], 'unique, nonmissing'),
    ([0, 1, 3], 'Run IDs do not match'),
])
def test_invalid_pairing_ids(ids, message):
    """Reject ambiguous or unmatched run identities."""
    a = pd.DataFrame({'run_idx': [0, 1, 2], 'x': [1, 2, 3]})
    b = pd.DataFrame({'run_idx': ids, 'x': [2, 3, 4]})
    with pytest.raises(ValueError, match=message):
        qq_plot.paired_differences(a, 'x', 'a', b, 'x', 'b')


@pytest.mark.parametrize('values, column, message', [
    ([1, 2, 3], 'missing', 'does not exist'),
    ([1, 'bad', 3], 'a', 'non-numeric'),
    ([1, np.inf, 3], 'a', 'infinite'),
    ([1, np.nan, 3], 'a', 'Fewer than 3'),
])
def test_invalid_selected_data(values, column, message):
    """Fail clearly for invalid values, absent columns, and too few pairs."""
    frame = pd.DataFrame({'a': values, 'b': [1, 2, 3]})
    with pytest.raises(ValueError, match=message):
        qq_plot.paired_differences(frame, column, 'input', column_b='b')


def test_missing_pairing_column():
    """Do not fall back to row order for separate files."""
    frame = pd.DataFrame({'x': [1, 2, 3]})
    with pytest.raises(ValueError, match='missing pairing column'):
        qq_plot.paired_differences(frame, 'x', 'a', frame, 'x', 'b')


def test_cli_saves_figure_and_prints_statistics(tmp_path, capsys):
    """Produce a real PNG and the sample standard deviation."""
    csv = tmp_path / 'results.csv'
    csv.write_text('a,b\n2,1\n4,2\n6,3\n')
    output = tmp_path / 'qq.png'
    assert qq_plot.main([str(csv), '--column-a', 'a', '--column-b', 'b',
                         '--output', str(output)]) == 0
    assert output.read_bytes().startswith(b'\x89PNG\r\n\x1a\n')
    text = capsys.readouterr().out
    assert 'Number of valid pairs: 3' in text
    assert 'Mean: 2' in text
    assert 'Median: 2' in text
    assert 'Standard deviation (sample, ddof=1): 1' in text
    assert 'Minimum: 1' in text
    assert 'Maximum: 3' in text


def test_interactive_plot_labels_and_constant_differences(monkeypatch):
    """Call show with correct labels and safely plot constant differences."""
    import matplotlib.pyplot as plt

    calls = []

    def capture_show():
        axes = plt.gcf().axes[0]
        calls.append(axes.get_title())
        assert axes.get_xlabel() == 'Theoretical quantiles'
        assert axes.get_ylabel() == 'Observed pairwise differences'
        np.testing.assert_array_equal(axes.lines[0].get_ydata(), [2, 2, 2])

    monkeypatch.setattr(plt, 'show', capture_show)
    qq_plot.plot_differences(np.array([2., 2., 2.]), 'A', 'B')
    assert calls == ['Q-Q Plot of Pairwise Differences: A - B']
    assert not plt.get_fignums()


def test_batch_keeps_campaigns_separate(tmp_path, campaign, capsys):
    """Plot all pairs per metric from each campaign's accepted rows."""
    for name in ('exp1', 'extended_exp1'):
        path = campaign(tmp_path / 'batch' / 'campaigns' / name)
        (path / 'unrelated.csv').write_text('invalid raw data')
    output = tmp_path / 'plots'
    assert qq_plot_batch.main([
        str(tmp_path / 'batch'), '--metrics', 'metric',
        '--output', str(output)]) == 0
    assert len(list(output.glob('exp1/q-q-plots/*.png'))) == 3
    assert len(list(output.glob('extended_exp1/q-q-plots/*.png'))) == 3
    text = capsys.readouterr().out
    assert 'Completed 6 plots; 0 failures.' in text
    # The failed attempt's 999 row would make a - b nonzero.
    assert 'a - b' in text and 'Maximum: 0' in text


def test_two_csvs_keep_accepted_attempts(tmp_path, campaign, capsys):
    """Pair two raw campaign CSVs on run_idx after attempt filtering."""
    path = campaign(tmp_path / 'campaign', ('a', 'b'))
    assert qq_plot.main([
        str(path / 'a_suave.csv'), str(path / 'b_suave.csv'),
        '--column-a', 'metric', '--column-b', 'metric',
        '--output', str(tmp_path / 'qq.png')]) == 0
    text = capsys.readouterr().out
    assert 'Number of valid pairs: 3' in text
    assert 'Maximum: 0' in text


def test_batch_reports_errors_and_continues(tmp_path, campaign, capsys):
    """Return failure while generating comparisons with available metrics."""
    path = campaign(tmp_path / 'campaign', ('a', 'b'))
    output = tmp_path / 'plots'
    assert qq_plot_batch.main([
        str(path), '--metrics', 'absent', 'metric',
        '--output', str(output)]) == 1
    assert len(list(output.glob('campaign/q-q-plots/*.png'))) == 1
    assert 'absent' in capsys.readouterr().err


def test_missing_file_cli(tmp_path, capsys):
    """Report a missing input path without a traceback."""
    assert qq_plot.main([str(tmp_path / 'missing.csv'),
                         '--column-a', 'a', '--column-b', 'b']) == 1
    assert 'does not exist' in capsys.readouterr().err


def test_batch_defaults_save_both_metrics_under_batch_root(
        tmp_path, metrics_campaign):
    """Save default metrics per campaign without specifying an output path."""
    for name in ('exp1', 'exp2'):
        metrics_campaign(tmp_path / 'campaigns' / name, {
            method: pd.DataFrame({
                'run_idx': [0, 1, 2],
                'time searching pipeline (s)': [1, 2, 4],
                'distance inspected (m)': [5, 3, 2],
            }) for method in ('a', 'b')})
    # Unrelated batch folders must never be mistaken for campaigns.
    (tmp_path / 'archived').mkdir()
    assert qq_plot_batch.main([str(tmp_path)]) == 0
    output = tmp_path / 'campaign_results'
    assert len(list((output / 'exp1').glob('q-q-plots/*.png'))) == 2
    assert len(list((output / 'exp2').glob('q-q-plots/*.png'))) == 2
    assert {p.name for p in output.iterdir()} == {'exp1', 'exp2'}
    # Output folders must not affect discovery on subsequent invocations.
    assert qq_plot_batch.main([str(tmp_path)]) == 0


@pytest.mark.parametrize('relative', ['', 'campaigns', 'campaigns/exp1'])
def test_batch_accepts_campaigns_dir_and_single_campaign(
        tmp_path, campaign, relative):
    """Keep the default result root consistent for narrower input paths."""
    campaign(tmp_path / 'campaigns' / 'exp1')
    assert qq_plot_batch.main([str(tmp_path / relative),
                               '--metrics', 'metric']) == 0
    assert len(list((tmp_path / 'campaign_results' / 'exp1' /
                     'q-q-plots').glob('*.png'))) == 3


def test_batch_missing_root(tmp_path, capsys):
    """Report a nonexistent batch before attempting to create outputs."""
    assert qq_plot_batch.main([str(tmp_path / 'missing')]) == 1
    assert 'not found' in capsys.readouterr().err


def test_csv_script_directs_folder_input_to_orchestrator(tmp_path, capsys):
    """Explain which script to use for batch input."""
    with pytest.raises(SystemExit) as error:
        qq_plot.main([str(tmp_path)])
    assert error.value.code == 2
    assert 'Use qq_plot_batch.py' in capsys.readouterr().err
