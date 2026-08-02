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

"""Summarize SUAVE campaign metrics and optionally create a LaTeX table."""

import argparse
from pathlib import Path

import pandas as pd


RESULT_FILE_SUFFIX = '_suave.csv'
DEFAULT_LATEX_FILENAME = 'suave_results.tex'
LATEX_CAPTION = 'SUAVE results mean and standard deviation.'
LATEX_LABEL = 'tab:suave_results'

METRICS = {
    'search_time': 'time searching pipeline (s)',
    'distance_inspected': 'distance inspected (m)',
    'reaction_time': 'mean reaction time (s)',
}
REQUIRED_COLUMNS = ['pipeline found', *METRICS.values()]
DISPLAY_NAMES = {
    'bt': 'BT',
    'metacontrol': 'Metacontrol',
    'none': 'None',
    'random': 'Random',
    'rebetmc': 'ReBeT-MC',
}
MANAGING_SYSTEM_ORDER = tuple(DISPLAY_NAMES)


def discover_result_files(results_path):
    """Find aggregate result CSV files and derive their system names."""
    results_path = Path(results_path).expanduser()
    if not results_path.is_dir():
        raise ValueError(f'Results directory does not exist: {results_path}')

    result_files = []
    for csv_file in results_path.glob(f'*{RESULT_FILE_SUFFIX}'):
        managing_system = csv_file.name[:-len(RESULT_FILE_SUFFIX)]
        result_files.append((managing_system, csv_file))

    if not result_files:
        raise ValueError(
            f'No *{RESULT_FILE_SUFFIX} files found in {results_path}')

    order = {name: index for index, name in enumerate(MANAGING_SYSTEM_ORDER)}
    return sorted(
        result_files,
        key=lambda item: (order.get(item[0], len(order)), item[0]))


def calculate_statistics(results_path):
    """Calculate campaign statistics for every discovered managing system."""
    summaries = []
    for managing_system, csv_file in discover_result_files(results_path):
        data = pd.read_csv(csv_file)
        missing_columns = [
            column for column in REQUIRED_COLUMNS if column not in data.columns
        ]
        if missing_columns:
            missing = ', '.join(missing_columns)
            raise ValueError(f'{csv_file} is missing columns: {missing}')
        if data.empty:
            raise ValueError(f'{csv_file} contains no result rows')

        metric_data = {}
        for metric_name, column in METRICS.items():
            try:
                values = pd.to_numeric(data[column], errors='raise')
            except (TypeError, ValueError) as error:
                raise ValueError(
                    f'{csv_file} contains non-numeric values in {column}'
                ) from error
            if values.isna().any():
                raise ValueError(
                    f'{csv_file} contains missing values in {column}')
            metric_data[metric_name] = values

        pipeline_found = _parse_pipeline_found(
            data['pipeline found'], csv_file)
        reaction_values = metric_data['reaction_time']
        reaction_values = reaction_values[reaction_values != 0]

        summaries.append({
            'managing_system': managing_system,
            'display_name': DISPLAY_NAMES.get(
                managing_system, managing_system.replace('_', ' ').title()),
            'runs': len(data),
            'pipelines_found': int(pipeline_found.sum()),
            'success_rate': float(pipeline_found.mean()),
            'search_time_mean': metric_data['search_time'].mean(),
            'search_time_std': metric_data['search_time'].std(ddof=1),
            'distance_inspected_mean': (
                metric_data['distance_inspected'].mean()),
            'distance_inspected_std': (
                metric_data['distance_inspected'].std(ddof=1)),
            'reaction_time_mean': reaction_values.mean(),
            'reaction_time_std': reaction_values.std(ddof=1),
            'reaction_time_samples': len(reaction_values),
        })

    return pd.DataFrame(summaries)


def format_console_table(summary, precision=2):
    """Format summary statistics as a terminal-friendly table."""
    rows = []
    for result in summary.to_dict('records'):
        rows.append({
            'Managing system': result['display_name'],
            'Pipeline found': _format_success_rate(result, precision),
            'Search time (s)': _format_statistic(
                result['search_time_mean'], result['search_time_std'],
                precision),
            'Distance inspected (m)': _format_statistic(
                result['distance_inspected_mean'],
                result['distance_inspected_std'], precision),
            'Reaction time (s)': _format_statistic(
                result['reaction_time_mean'], result['reaction_time_std'],
                precision),
        })
    return pd.DataFrame(rows).to_string(index=False)


def render_latex_table(summary, precision=2):
    """Render summary statistics as a complete LaTeX table environment."""
    lines = [
        r'\begin{table}[htbp]',
        r'\centering',
        r'\resizebox{\linewidth}{!}{%',
        r'\begin{tabular}{lcccc}',
        r'\toprule',
        ('Managing system & Pipeline found & Search time (s) & '
         r'Distance inspected (m) & Reaction time (s) \\'),
        r'\midrule',
    ]

    for result in summary.to_dict('records'):
        display_name = _escape_latex(result['display_name'])
        success_rate = _format_success_rate(result, precision, latex=True)
        search_time = _format_statistic(
            result['search_time_mean'], result['search_time_std'],
            precision, latex=True)
        distance = _format_statistic(
            result['distance_inspected_mean'],
            result['distance_inspected_std'], precision, latex=True)
        reaction_time = _format_statistic(
            result['reaction_time_mean'], result['reaction_time_std'],
            precision, latex=True)
        lines.append(
            f'{display_name} & {success_rate} & {search_time} & '
            f'{distance} & {reaction_time} ' + r'\\')

    lines.extend([
        r'\bottomrule',
        r'\end{tabular}%',
        '}',
        f'\\caption{{{LATEX_CAPTION}}}\\label{{{LATEX_LABEL}}}',
        r'\end{table}',
        '',
    ])
    return '\n'.join(lines)


def write_latex_table(summary, output_path, precision=2):
    """Write a rendered LaTeX table to the requested file."""
    output_path = Path(output_path).expanduser()
    if output_path.suffix == '':
        output_path = output_path.with_suffix('.tex')
    elif output_path.suffix != '.tex':
        raise ValueError('LaTeX output file must use the .tex extension')

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        render_latex_table(summary, precision), encoding='utf-8')
    return output_path


def build_argument_parser():
    """Create the command-line argument parser."""
    parser = argparse.ArgumentParser(
        description=(
            'Calculate mean and sample standard deviation for SUAVE results.'))
    parser.add_argument(
        'results_path', type=Path,
        help='campaign folder containing *_suave.csv result files')
    parser.add_argument(
        '--latex', action='store_true',
        help='write a LaTeX table to the campaign folder')
    parser.add_argument(
        '--latex-output', type=Path,
        help=('custom LaTeX output file; specifying it also enables LaTeX '
              'output'))
    parser.add_argument(
        '--precision', type=int, default=2,
        help=('number of decimal places in printed and LaTeX values '
              '(default: 2)'))
    return parser


def main(args=None):
    """Run the result summary command-line program."""
    parser = build_argument_parser()
    parsed_args = parser.parse_args(args=args)
    if parsed_args.precision < 0:
        parser.error('--precision must be zero or greater')

    try:
        summary = calculate_statistics(parsed_args.results_path)
        print(format_console_table(summary, parsed_args.precision))

        if parsed_args.latex or parsed_args.latex_output is not None:
            output_path = parsed_args.latex_output
            if output_path is None:
                output_path = (
                    parsed_args.results_path.expanduser() /
                    DEFAULT_LATEX_FILENAME)
            output_path = write_latex_table(
                summary, output_path, parsed_args.precision)
            print(f'LaTeX table written to {output_path}')
    except ValueError as error:
        parser.error(str(error))


def _parse_pipeline_found(values, csv_file):
    normalized = values.map(
        lambda value: str(value).strip().lower() if not pd.isna(value) else '')
    true_values = {'true', '1', 'yes'}
    false_values = {'false', '0', 'no'}
    invalid = ~normalized.isin(true_values | false_values)
    if invalid.any():
        raise ValueError(f'{csv_file} contains invalid pipeline found values')
    return normalized.isin(true_values)


def _format_statistic(mean, standard_deviation, precision, latex=False):
    if pd.isna(mean):
        return 'N/A'
    formatted_mean = f'{mean:.{precision}f}'
    formatted_std = (
        'N/A' if pd.isna(standard_deviation)
        else f'{standard_deviation:.{precision}f}')
    if latex:
        if formatted_std == 'N/A':
            formatted_std = r'\mathrm{N/A}'
        return f'${formatted_mean} \\pm {formatted_std}$'
    return f'{formatted_mean} +/- {formatted_std}'


def _format_success_rate(result, precision, latex=False):
    percentage = result['success_rate'] * 100
    percent_sign = r'\%' if latex else '%'
    return (
        f'{result["pipelines_found"]}/{result["runs"]} '
        f'({percentage:.{precision}f}{percent_sign})')


def _escape_latex(value):
    replacements = {
        '\\': r'\textbackslash{}',
        '&': r'\&',
        '%': r'\%',
        '$': r'\$',
        '#': r'\#',
        '_': r'\_',
        '{': r'\{',
        '}': r'\}',
        '~': r'\textasciitilde{}',
        '^': r'\textasciicircum{}',
    }
    return ''.join(
        replacements[character] if character in replacements else character
        for character in str(value))


if __name__ == '__main__':
    main()
