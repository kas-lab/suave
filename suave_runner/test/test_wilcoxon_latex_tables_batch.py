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

"""Verify batch LaTeX rendering, p-value selection, and input validation."""

import importlib.util
from pathlib import Path
import sys

import numpy as np

import pandas as pd

import pytest


SOURCE = Path(__file__).parents[1] / 'suave_runner' / 'latex'
SPEC = importlib.util.spec_from_file_location(
    'latex_tables', SOURCE / 'latex_tables.py')
renderer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(renderer)
sys.modules['latex_tables'] = renderer
SPEC = importlib.util.spec_from_file_location(
    'wilcoxon_latex_tables_batch', SOURCE / 'wilcoxon_latex_tables_batch.py')
batch = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(batch)


def _results():
    rows = []
    for a, b in [('bt', 'planta'), ('planta', 'bt')]:
        for metric, alternative in batch.METRIC_ALTERNATIVES.items():
            rows.append({
                'row': a, 'col': b, 'metric': metric,
                'alternative': alternative,
                'p_raw': 0.01 if a == 'bt' else 0.9,
                'p_adjusted': 0.08 if a == 'bt' else 1.0,
                'correction': 'holm', 'note': '',
            })
    return pd.DataFrame(rows)


def _campaign(root, name='exp1'):
    (root / 'campaigns' / name).mkdir(parents=True)
    folder = root / 'campaings_results' / name / 'wilcoxon_analysis'
    folder.mkdir(parents=True)
    path = folder / f'{name}_wilcoxon_results.csv'
    _results().to_csv(path, index=False)
    return path


def test_default_batch_tables_use_adjusted_values(tmp_path):
    """Render every campaign using corrected values in the requested folder."""
    paths = [_campaign(tmp_path, name) for name in ('exp1', 'extended_exp1')]
    originals = {p: p.read_bytes() for p in paths}
    assert batch.main([str(tmp_path)]) == 0
    folder = tmp_path / 'campaigns_latex_tables' / 'wilcoxon_analysis'
    assert {p.name for p in folder.glob('*.tex')} == {
        'exp1_wilcoxon.tex', 'extended_exp1_wilcoxon.tex'}
    text = (folder / 'exp1_wilcoxon.tex').read_text()
    assert r'\cellcolor{pvalue_blue}0.080' in text
    assert '0.010' not in text
    assert 'Holm-adjusted' in text
    assert r'\textbf{BT} & - & \cellcolor{pvalue_blue}0.080' in text
    assert r'\label{tab:wilcoxon-exp1}' in text
    assert originals == {p: p.read_bytes() for p in paths}


def test_raw_values_require_explicit_option(tmp_path):
    """Render raw values and identify them as unadjusted in the caption."""
    _campaign(tmp_path)
    output = tmp_path / 'custom'
    assert batch.main([str(tmp_path), '--p-values', 'raw',
                       '--output', str(output)]) == 0
    text = (output / 'exp1_wilcoxon.tex').read_text()
    assert r'\cellcolor{pvalue_green}0.010' in text
    assert 'Unadjusted p-values' in text
    assert 'Holm-adjusted' not in text


def test_uncomputed_tests_and_escaped_labels(tmp_path):
    """Preserve absent tests and escape unknown method labels."""
    path = _campaign(tmp_path)
    frame = _results().replace({'planta': 'method_A&B'})
    frame.loc[0, ['p_raw', 'p_adjusted']] = np.nan
    frame.loc[0, 'note'] = 'fewer than 2 nonzero differences; not computed'
    frame.to_csv(path, index=False)
    assert batch.main([str(tmp_path)]) == 0
    table = (tmp_path / 'campaigns_latex_tables' / 'wilcoxon_analysis' /
             'exp1_wilcoxon.tex').read_text()
    assert r'method\_A\&B' in table
    assert r'\textbf{BT} & - & -' in table
    assert 'nan' not in table


@pytest.mark.parametrize('problem, message', [
    ('missing_column', 'missing columns'),
    ('missing_pair', 'incomplete or unexpected'),
    ('duplicate', 'duplicate'),
    ('invalid_p', 'between 0 and 1'),
    ('invalid_text', 'non-numeric'),
    ('missing_adjustment', 'availability differs'),
    ('missing_note', 'without explanatory notes'),
    ('wrong_alternative', 'unexpected alternative'),
    ('mixed_correction', 'single holm/none'),
])
def test_invalid_results_are_rejected(tmp_path, problem, message):
    """Reject inconsistent comparisons instead of silently making a table."""
    frame = _results()
    if problem == 'missing_column':
        frame = frame.drop(columns='p_adjusted')
    elif problem == 'missing_pair':
        frame = frame.iloc[:-1]
    elif problem == 'duplicate':
        frame = pd.concat([frame, frame.iloc[[0]]])
    elif problem == 'invalid_p':
        frame.loc[0, 'p_adjusted'] = 1.1
    elif problem == 'invalid_text':
        frame.loc[0, 'p_adjusted'] = 'invalid'
    elif problem == 'missing_adjustment':
        frame.loc[0, 'p_adjusted'] = np.nan
    elif problem == 'missing_note':
        frame.loc[0, ['p_raw', 'p_adjusted']] = np.nan
    elif problem == 'wrong_alternative':
        frame.loc[0, 'alternative'] = 'two-sided'
    elif problem == 'mixed_correction':
        frame.loc[0, 'correction'] = 'none'
    path = tmp_path / 'results.csv'
    frame.to_csv(path, index=False)
    with pytest.raises(ValueError, match=message):
        batch.load_results(path)


def test_missing_analysis_does_not_skip_silently(tmp_path, capsys):
    """Report missing analysis while still rendering available campaigns."""
    _campaign(tmp_path, 'good')
    (tmp_path / 'campaigns' / 'bad').mkdir()
    assert batch.main([str(tmp_path)]) == 1
    assert 'run wilcoxon_analysis_batch.py first' in capsys.readouterr().err
    assert (tmp_path / 'campaigns_latex_tables' / 'wilcoxon_analysis' /
            'good_wilcoxon.tex').is_file()


def test_custom_results_root(tmp_path):
    """Locate analysis produced with an overridden batch output root."""
    _campaign(tmp_path)
    custom = tmp_path / 'custom_results'
    (tmp_path / 'campaings_results').rename(custom)
    assert batch.main([str(tmp_path), '--results-root', str(custom)]) == 0


def test_missing_batch(tmp_path, capsys):
    """Report an absent batch without a traceback."""
    assert batch.main([str(tmp_path / 'missing')]) == 1
    assert 'does not exist' in capsys.readouterr().err
