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

"""Render pairwise p-value matrices as LaTeX tables."""

# wilcoxon_analysis as LaTeX.
#
# This module has no ROS dependency: it only reads the matrix CSVs written by
# `StatisticalAnalysis.save_data_frames()` and formats them into the two-metric
# pairwise-comparison table style used in the PLANTA manuscript
# (`\cellcolor{pvalue_green}` / `\cellcolor{pvalue_blue}` cells, one block of
# columns per metric). Run standalone: `python3 latex_tables.py --config
# table.json`.

import argparse
import json
from pathlib import Path

import pandas as pd

ALPHA_DEFAULT = 0.05


def _cell(value, alpha):
    """Format one p-value as a colored LaTeX cell, or '-' for the diagonal."""
    if pd.isna(value):
        return '-'
    color = 'pvalue_green' if value < alpha else 'pvalue_blue'
    formatted = f'{value:.3f}'
    if formatted == '0.000':
        formatted = r'$<0.001$'
    return f'\\cellcolor{{{color}}}{formatted}'


def _load_matrix(csv_path, keys):
    """Load a p-value matrix CSV and reorder it to the requested key order."""
    matrix = pd.read_csv(csv_path, index_col=0)
    return matrix.loc[keys, keys]


def build_table_tex(config):
    """Build the full LaTeX table string described by a table config dict."""
    keys = [system['key'] for system in config['systems']]
    labels = [system['label'] for system in config['systems']]
    alpha = config.get('alpha', ALPHA_DEFAULT)
    n = len(keys)

    search_matrix = _load_matrix(config['time_search_csv'], keys)
    distance_matrix = _load_matrix(config['distance_csv'], keys)

    col_spec = (
        f"p{{{config['first_col_width']}}}"
        f"|*{{{n}}}{{p{{{config['col_width']}}}}}"
        f"|*{{{n}}}{{p{{{config['col_width']}}}}}"
    )
    header_labels = ' & '.join(f'\\textbf{{{label}}}' for label in labels)

    rows = []
    for row_key, row_label in zip(keys, labels):
        search_cells = ' & '.join(
            _cell(search_matrix.loc[row_key, col_key], alpha)
            for col_key in keys
        )
        distance_cells = ' & '.join(
            _cell(distance_matrix.loc[row_key, col_key], alpha)
            for col_key in keys
        )
        rows.append(
            f'\\textbf{{{row_label}}} & {search_cells} & {distance_cells} '
            '\\\\')
    rows_tex = '\n'.join(rows)
    table_header = (
        f"\\multirow{{2}}{{{config['header_width']}}}"
        '{\\raggedright \\textbf{Managing subsystem}} & '
        f'\\multicolumn{{{n}}}{{c|}}{{\\textbf{{Pipeline search time}}}} & '
        f'\\multicolumn{{{n}}}{{c}}'
        '{\\textbf{Pipeline distance inspected}} \\\\ '
        f'\\cline{{2-{2 * n + 1}}}')

    return f"""\\begin{{table}}[tb]
\\centering
\\caption{{{config['caption']}}}
\\label{{{config['label']}}}
\\renewcommand{{\\arraystretch}}{{1.2}} % Row height for better readability
\\resizebox{{{config['resizebox_width']}}}{{!}}{{%
\\begin{{tabular}}{{{col_spec}}}
\\hline
{table_header}
 & {header_labels} & {header_labels} \\\\
\\hline
{rows_tex}
\\hline
\\end{{tabular}}
}}
\\end{{table}}
"""


def generate_table_file(config):
    """Build one table and write it to the config's output path."""
    tex = build_table_tex(config)
    output_path = Path(config['output']).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(tex, encoding='utf-8')
    return output_path


def main(args=None):
    """Generate a LaTeX pairwise p-value table from a JSON config file."""
    parser = argparse.ArgumentParser(
        description=(
            'Render a pairwise p-value matrix (search time + distance '
            'inspected) as a LaTeX table.'))
    parser.add_argument(
        '--config', required=True,
        help='Path to a JSON table config (see module docstring for keys).')
    parsed = parser.parse_args(args)

    config = json.loads(Path(parsed.config).expanduser().read_text())
    output_path = generate_table_file(config)
    print(f'Wrote {output_path}')


if __name__ == '__main__':
    main()
