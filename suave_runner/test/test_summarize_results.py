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

"""Tests for the SUAVE campaign result summary command."""

from pathlib import Path

import pandas as pd

import pytest

from suave_runner.summarize_results import calculate_statistics
from suave_runner.summarize_results import main
from suave_runner.summarize_results import render_latex_table


def _write_results(path, managing_system, rows):
    pd.DataFrame(rows).to_csv(
        path / f'{managing_system}_suave.csv', index=False)


def _result_rows(reaction_times=(0.0, 2.0, 4.0)):
    return {
        'pipeline found': [True, False, True],
        'time searching pipeline (s)': [10.0, 14.0, 12.0],
        'distance inspected (m)': [20.0, 24.0, 22.0],
        'mean reaction time (s)': reaction_times,
    }


def test_calculate_statistics_uses_all_runs_and_nonzero_reactions(tmp_path):
    """Use every run except zero-valued reaction-time samples."""
    _write_results(tmp_path, 'bt', _result_rows())

    result = calculate_statistics(tmp_path).iloc[0]

    assert result['managing_system'] == 'bt'
    assert result['display_name'] == 'BT'
    assert result['runs'] == 3
    assert result['pipelines_found'] == 2
    assert result['success_rate'] == pytest.approx(2 / 3)
    assert result['search_time_mean'] == pytest.approx(12.0)
    assert result['search_time_std'] == pytest.approx(2.0)
    assert result['distance_inspected_mean'] == pytest.approx(22.0)
    assert result['distance_inspected_std'] == pytest.approx(2.0)
    assert result['reaction_time_mean'] == pytest.approx(3.0)
    assert result['reaction_time_std'] == pytest.approx(2 ** 0.5)
    assert result['reaction_time_samples'] == 2


def test_calculate_statistics_uses_na_when_no_reactions_exist(tmp_path):
    """Report unavailable reaction statistics when all values are zero."""
    _write_results(tmp_path, 'none', _result_rows((0.0, 0.0, 0.0)))

    result = calculate_statistics(tmp_path).iloc[0]

    assert pd.isna(result['reaction_time_mean'])
    assert pd.isna(result['reaction_time_std'])
    assert result['reaction_time_samples'] == 0


def test_render_latex_table_formats_names_rates_and_na(tmp_path):
    """Render publication names, success rates, and unavailable values."""
    _write_results(tmp_path, 'none', _result_rows((0.0, 0.0, 0.0)))
    _write_results(tmp_path, 'rebetmc', _result_rows())

    latex = render_latex_table(calculate_statistics(tmp_path))

    assert r'\caption{SUAVE results mean and standard deviation.}' in latex
    assert r'\label{tab:suave_results}' in latex
    assert r'\resizebox{\linewidth}{!}{%' in latex
    assert latex.index(r'\resizebox{\linewidth}{!}{%') < latex.index(
        r'\caption{SUAVE results mean and standard deviation.}')
    assert r'None & 2/3 (66.67\%)' in latex
    assert 'None & 2/3 (66.67\\%) & $12.00' in latex
    assert '& N/A ' in latex
    assert 'ReBeT-MC' in latex


def test_main_writes_default_latex_file(tmp_path, capsys):
    """Write suave_results.tex to the campaign folder by default."""
    _write_results(tmp_path, 'bt', _result_rows())

    main([str(tmp_path), '--latex'])

    output_path = tmp_path / 'suave_results.tex'
    assert output_path.is_file()
    assert 'BT & 2/3 (66.67\\%)' in output_path.read_text(encoding='utf-8')
    assert f'LaTeX table written to {output_path}' in capsys.readouterr().out


def test_latex_output_implies_latex_generation(tmp_path):
    """Enable LaTeX generation when a custom output path is supplied."""
    _write_results(tmp_path, 'bt', _result_rows())
    output_path = tmp_path / 'tables' / 'custom.tex'

    main([str(tmp_path), '--latex-output', str(output_path)])

    assert output_path.is_file()


def test_calculate_statistics_rejects_missing_columns(tmp_path):
    """Reject aggregate files that omit required metric columns."""
    pd.DataFrame({'pipeline found': [True]}).to_csv(
        tmp_path / 'bt_suave.csv', index=False)

    with pytest.raises(ValueError, match='missing columns'):
        calculate_statistics(tmp_path)


def test_calculate_statistics_ignores_nonaggregate_csv_files(tmp_path):
    """Ignore per-event CSV files in the campaign folder."""
    _write_results(tmp_path, 'bt', _result_rows())
    Path(tmp_path / 'bt_suave_wv_reaction_time.csv').write_text(
        'reaction time (s)\n1.0\n', encoding='utf-8')

    summary = calculate_statistics(tmp_path)

    assert summary['managing_system'].tolist() == ['bt']
