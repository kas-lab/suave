#!/usr/bin/env python3
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

"""Generate all campaign Wilcoxon LaTeX tables with the existing renderer."""

import argparse
from pathlib import Path
import re
import sys
from tempfile import TemporaryDirectory

import numpy as np

import pandas as pd


if __package__:
    from . import latex_tables
else:
    import latex_tables


HELP_TEXT = """Generate all campaign Wilcoxon LaTeX tables with the existing renderer.

Keep latex_tables.py alongside this script. No ROS imports are required.

    python3 wilcoxon_latex_tables_batch.py /path/to/batch
    python3 wilcoxon_latex_tables_batch.py /path/to/batch --p-values raw

Reads campaings_results/<experiment>/wilcoxon_analysis/*_results.csv.
Writes campaigns_latex_tables/wilcoxon_analysis/<experiment>_wilcoxon.tex.
Adjusted p-values are used by default. Analysis must already exist: this
script renders results without recomputing tests or sorting raw data.
"""


METHOD_LABELS = {
    'none': 'None', 'random': 'Random', 'bt': 'BT',
    'metacontrol': 'MC', 'rosa_bt': 'ROSA', 'planta': 'PLANTA',
}
METRIC_ALTERNATIVES = {'search_time': 'less', 'distance_inspected': 'greater'}
TEX_ESCAPES = {
    '\\': r'\textbackslash{}', '&': r'\&', '%': r'\%', '$': r'\$',
    '#': r'\#', '_': r'\_', '{': r'\{', '}': r'\}',
    '~': r'\textasciitilde{}', '^': r'\textasciicircum{}',
}


def escape_tex(text):
    """Escape literal labels for safe inclusion in LaTeX table text."""
    return ''.join(TEX_ESCAPES.get(character, character)
                   for character in str(text))


def load_results(path):
    """Validate complete ordered-pair results and their p-value provenance."""
    try:
        frame = pd.read_csv(path)
    except (OSError, UnicodeError, pd.errors.ParserError,
            pd.errors.EmptyDataError) as error:
        raise ValueError(f'Cannot read {path}: {error}') from error
    required = {'row', 'col', 'metric', 'alternative', 'p_raw',
                'p_adjusted', 'correction', 'note'}
    missing = required - set(frame.columns)
    if missing:
        raise ValueError(f'{path}: missing columns {sorted(missing)}')
    if frame.empty:
        raise ValueError(f'{path}: no comparisons')
    for column in ('row', 'col'):
        valid = frame[column].map(
            lambda value: isinstance(value, str) and bool(value.strip()))
        if not valid.all():
            raise ValueError(f'{path}: missing or invalid method labels')
    systems = set(frame['row']) | set(frame['col'])
    if len(systems) < 2:
        raise ValueError(f'{path}: need at least two methods')
    if frame.duplicated(['row', 'col', 'metric']).any():
        raise ValueError(f'{path}: duplicate method-pair/metric comparisons')
    expected = {(a, b, metric) for a in systems for b in systems if a != b
                for metric in METRIC_ALTERNATIVES}
    actual = set(frame[['row', 'col', 'metric']].itertuples(
        index=False, name=None))
    if actual != expected:
        raise ValueError(
            f'{path}: incomplete or unexpected comparisons '
            f'({len(expected - actual)} missing, '
            f'{len(actual - expected)} extra)')
    for metric, alternative in METRIC_ALTERNATIVES.items():
        if not (frame.loc[frame.metric == metric, 'alternative'] ==
                alternative).all():
            raise ValueError(f'{path}: unexpected alternative for {metric}')
    corrections = frame['correction'].unique()
    if len(corrections) != 1 or corrections[0] not in ('holm', 'none'):
        raise ValueError(
            f'{path}: expected a single holm/none correction policy')
    for column in ('p_raw', 'p_adjusted'):
        try:
            values = pd.to_numeric(frame[column], errors='raise')
        except (TypeError, ValueError) as error:
            raise ValueError(f'{path}: non-numeric {column}') from error
        finite = values.dropna().to_numpy(dtype=float)
        if (not np.isfinite(finite).all() or
                ((finite < 0) | (finite > 1)).any()):
            raise ValueError(f'{path}: {column} must be between 0 and 1')
        frame[column] = values
    absent = frame['p_raw'].isna()
    if not absent.equals(frame['p_adjusted'].isna()):
        raise ValueError(f'{path}: raw/adjusted p-value availability differs')
    notes = frame['note'].fillna('').astype(str).str.strip()
    if (absent & notes.eq('')).any():
        raise ValueError(f'{path}: missing p-values without explanatory notes')
    return frame


CAMPAIGN_NAME_PATTERN = re.compile(r'^(extended_)?exp(\d+)$')


def _campaign_title(name):
    """Return the manuscript-style title for a campaign, e.g. 'Experiment 1'."""
    match = CAMPAIGN_NAME_PATTERN.match(name)
    if not match:
        return name.replace('_', ' ')
    extended, number = match.groups()
    prefix = 'Extended experiment' if extended else 'Experiment'
    return f'{prefix} {number}'


def render_campaign(campaign, results_root, output_root, p_values, alpha):
    """Create one two-metric table using the existing matrix renderer."""
    folder = results_root / campaign.name / 'wilcoxon_analysis'
    inputs = sorted(folder.glob('*_results.csv'))
    if len(inputs) != 1:
        raise ValueError(
            f'{folder}: expected one *_results.csv, found {len(inputs)}; '
            'run wilcoxon_analysis_batch.py first if analysis is missing')
    frame = load_results(inputs[0])
    keys_present = set(frame['row']) | set(frame['col'])
    keys = [key for key in METHOD_LABELS if key in keys_present]
    keys += sorted(keys_present - set(keys))
    correction = frame['correction'].iloc[0]
    value_column = 'p_adjusted' if p_values == 'adjusted' else 'p_raw'
    p_value_desc = ('Holm-adjusted p-values'
                    if p_values == 'adjusted' and correction == 'holm' else
                    'Unadjusted p-values')
    title = escape_tex(_campaign_title(campaign.name))
    caption = (
        f'{title}: paired Wilcoxon signed-rank comparisons. Each cell '
        f'reports the {p_value_desc} for the comparison between the '
        'managing subsystem in the row and the one in the column for both '
        r'the \emph{pipeline search time} and \emph{pipeline distance '
        r'inspected} metrics. Cells with a green background indicate that '
        'the alternative hypothesis was favored, and cells with a blue '
        'background indicate that the null hypothesis cannot be rejected '
        'in favor of the alternative hypothesis. Values rounding to 0.000 '
        'are shown as $<0.001$.')
    if 'metacontrol' in keys:
        caption += " ``MC'' stands for Metacontrol."
    output = output_root / f'{campaign.name}_wilcoxon.tex'
    with TemporaryDirectory(prefix='wilcoxon-latex-') as temporary:
        matrices = {}
        for metric in METRIC_ALTERNATIVES:
            rows = frame.loc[frame.metric == metric]
            matrix = rows.pivot(
                index='row', columns='col', values=value_column)
            matrix = matrix.reindex(index=keys, columns=keys)
            path = Path(temporary) / f'{metric}.csv'
            matrix.to_csv(path)
            matrices[metric] = str(path)
        config = {
            'systems': [{'key': key, 'label': escape_tex(
                METHOD_LABELS.get(key, key))} for key in keys],
            'time_search_csv': matrices['search_time'],
            'distance_csv': matrices['distance_inspected'],
            'first_col_width': '1.65cm', 'col_width': '1.3cm',
            'header_width': '1.65cm',
            'resizebox_width': (r'0.7\textwidth' if len(keys) <= 3
                                else r'\textwidth'),
            'caption': caption,
            'label': 'tab:wilcoxon-' + re.sub(
                r'[^A-Za-z0-9-]', '-', campaign.name),
            'alpha': alpha, 'output': str(output),
        }
        latex_tables.generate_table_file(config)
    print(f'Wrote {output} ({value_column}; '
          f'{int(frame[value_column].isna().sum())} uncomputed tests)')
    return output


def build_parser():
    """Configure batch discovery, p-values, and output directory."""
    parser = argparse.ArgumentParser(
        description=HELP_TEXT,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('input', type=Path, help='Batch root with campaigns/')
    parser.add_argument('--results-root', type=Path,
                        help='Analysis root; defaults to '
                        '<batch>/campaings_results')
    parser.add_argument('--output', type=Path,
                        help='Table directory; defaults to '
                        '<batch>/campaigns_latex_tables/wilcoxon_analysis')
    parser.add_argument('--p-values', choices=('adjusted', 'raw'),
                        default='adjusted', help='Values to display '
                        '(default: adjusted; no correction is recomputed)')
    parser.add_argument('--alpha', type=float, default=0.05,
                        help='Cell-color threshold (default: 0.05)')
    return parser


def main(argv=None):
    """Render independent campaign tables and report failures explicitly."""
    parser = build_parser()
    args = parser.parse_args(argv)
    if not 0 < args.alpha < 1:
        parser.error('--alpha must be between 0 and 1')
    root = args.input.expanduser().resolve()
    results_root = (args.results_root.expanduser() if args.results_root else
                    root / 'campaings_results')
    output_root = (args.output.expanduser() if args.output else
                   root / 'campaigns_latex_tables' / 'wilcoxon_analysis')
    try:
        if not (root / 'campaigns').is_dir():
            raise ValueError(f'Campaigns directory does not exist: {root}')
        campaigns = sorted(p for p in (root / 'campaigns').iterdir()
                           if p.is_dir())
        if not campaigns:
            raise ValueError(f'No campaigns found in {root / "campaigns"}')
    except (ValueError, OSError) as error:
        print(f'Error: {error}', file=sys.stderr)
        return 1
    failures = 0
    for campaign in campaigns:
        try:
            render_campaign(campaign, results_root, output_root,
                            args.p_values, args.alpha)
        except (ValueError, OSError, KeyError) as error:
            print(f'Error: {campaign.name}: {error}', file=sys.stderr)
            failures += 1
    print(f'Generated {len(campaigns) - failures} tables; '
          f'{failures} failures.')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
