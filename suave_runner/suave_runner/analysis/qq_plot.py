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

"""Plot normal Q-Q plots of paired differences, without normality tests."""

import argparse
from pathlib import Path
import sys

import matplotlib

import numpy as np

import pandas as pd

from scipy import stats


HELP_TEXT = r"""Plot normal Q-Q plots of paired differences, without normality tests.

Requires pandas, numpy, scipy, and matplotlib; runs without ROS imports.
Run from the directory containing this script, for example:

    python3 qq_plot.py results.csv --column-a suave --column-b baseline \
        --output qq_suave_baseline.png

    python3 qq_plot.py planta_sorted.csv bt_sorted.csv \
        --column-a 'time searching pipeline (s)' \
        --column-b 'time searching pipeline (s)' --output qq_search.png

One CSV pairs columns by row. Two CSVs pair by unique run_idx values and
must describe the same campaign. Use qq_plot_batch.py to process a batch
and save every within-campaign comparison to campaings_results.
All complete pairs are included, even if pipeline found is false or a
metric is zero. No automatic normal/non-normal classification is made.
"""


def read_csv(path):
    """Read a nonempty CSV with an informative error on failure."""
    path = Path(path).expanduser()
    if not path.is_file():
        raise ValueError(f'Input CSV does not exist or is not a file: {path}')
    try:
        return pd.read_csv(path)
    except (OSError, UnicodeError, pd.errors.ParserError,
            pd.errors.EmptyDataError) as error:
        raise ValueError(f'Cannot read CSV {path}: {error}') from error


def numeric_column(frame, column, source):
    """Validate a column, retaining missing values for paired removal."""
    if column not in frame.columns:
        raise ValueError(
            f'{source}: column {column!r} does not exist. '
            f'Available columns: {list(frame.columns)}')
    values = frame[column]
    try:
        numeric = pd.to_numeric(values, errors='raise').astype(float)
    except (ValueError, TypeError, OverflowError) as error:
        raise ValueError(
            f'{source}: column {column!r} contains non-numeric values: '
            f'{error}') from error
    if np.isinf(numeric.to_numpy()).any():
        raise ValueError(
            f'{source}: column {column!r} contains infinite values.')
    return numeric


def validate_run_ids(frame, run_column, source):
    """Require nonmissing, unique run identifiers before joining two files."""
    if run_column not in frame.columns:
        raise ValueError(
            f'{source}: missing pairing column {run_column!r}; use sorted '
            'CSV files with verified run IDs or specify --run-column.')
    ids = frame[run_column]
    if ids.isna().any() or ids.duplicated().any():
        raise ValueError(
            f'{source}: {run_column!r} must contain unique, nonmissing IDs.')
    return ids


def paired_differences(frame_a, column_a, source_a, frame_b=None,
                       column_b=None, source_b=None, run_column='run_idx'):
    """Align pairs, remove incomplete pairs, and compute A minus B."""
    if frame_b is None:
        frame_b, source_b = frame_a, source_a
        ids_a = ids_b = None
    else:
        ids_a = validate_run_ids(frame_a, run_column, source_a)
        ids_b = validate_run_ids(frame_b, run_column, source_b)
        only_a = set(ids_a) - set(ids_b)
        only_b = set(ids_b) - set(ids_a)
        if only_a or only_b:
            raise ValueError(
                f'Run IDs do not match between {source_a} and {source_b}: '
                f'{len(only_a)} only in A, {len(only_b)} only in B. '
                'Refusing to silently discard unmatched runs.')
    a = numeric_column(frame_a, column_a, source_a)
    b = numeric_column(frame_b, column_b, source_b)
    if ids_a is None:
        pairs = pd.DataFrame({'a': a, 'b': b})
    else:
        left = pd.DataFrame({'run': ids_a, 'a': a})
        right = pd.DataFrame({'run': ids_b, 'b': b})
        pairs = left.merge(right, on='run', validate='one_to_one')
    complete = pairs.dropna(subset=['a', 'b'])
    dropped = len(pairs) - len(complete)
    if len(complete) < 3:
        raise ValueError(
            f'Fewer than 3 valid paired observations: {len(complete)} '
            f'remain after removing {dropped} incomplete pairs.')
    with np.errstate(over='ignore', invalid='ignore'):
        difference = (complete['a'] - complete['b']).to_numpy()
    if not np.isfinite(difference).all():
        raise ValueError('Subtraction produced non-finite differences.')
    return difference, dropped


def describe_differences(difference, label_a, label_b, dropped, context=''):
    """Print descriptive statistics, using the sample standard deviation."""
    heading = f'{label_a} - {label_b}'
    print(f'\n{context + ": " if context else ""}{heading}')
    print(f'Number of valid pairs: {len(difference)}')
    print(f'Incomplete pairs removed: {dropped}')
    for label, value in (
        ('Mean', np.mean(difference)),
        ('Median', np.median(difference)),
        ('Standard deviation (sample, ddof=1)', np.std(difference, ddof=1)),
        ('Minimum', np.min(difference)),
        ('Maximum', np.max(difference)),
    ):
        print(f'{label}: {value:.6g}')


def plot_differences(difference, label_a, label_b, output=None, context=''):
    """Display or save a normal Q-Q plot with a fitted reference line."""
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(figsize=(9, 6))
    try:
        theoretical, observed = stats.probplot(
            difference, dist='norm', fit=False, plot=axes)
        # Fit a reference line without computing a correlation or test.
        if np.ptp(observed) == 0:
            reference = np.full_like(theoretical, observed[0])
        else:
            slope, intercept = np.polyfit(theoretical, observed, 1)
            reference = slope * theoretical + intercept
        axes.plot(theoretical, reference, color='tab:red',
                  label='Fitted reference line')
        title = f'Q-Q Plot of Pairwise Differences: {label_a} - {label_b}'
        if context:
            title += f'\n{context}'
        axes.set_title(title, fontsize=11, wrap=True)
        axes.set_xlabel('Theoretical quantiles')
        axes.set_ylabel('Observed pairwise differences')
        axes.legend()
        axes.grid(alpha=0.25)
        figure.tight_layout()
        if output is None:
            plt.show()
        else:
            output = Path(output).expanduser()
            if not output.suffix:
                raise ValueError(
                    '--output needs a figure extension, e.g. .png')
            output.parent.mkdir(parents=True, exist_ok=True)
            figure.savefig(output)
            print(f'Saved: {output}')
    finally:
        plt.close(figure)


def method_name(path):
    """Derive a method label from an aggregate result filename."""
    name = Path(path).stem
    for suffix in ('_sorted', '_suave_extended', '_suave'):
        if name.endswith(suffix):
            name = name[:-len(suffix)]
    return name


def build_parser():
    """Create the command-line interface for one or two CSV files."""
    parser = argparse.ArgumentParser(
        description=HELP_TEXT,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('input', type=Path,
                        help='First input CSV')
    parser.add_argument('csv_b', type=Path, nargs='?',
                        help='Second CSV; pairs files by run ID')
    parser.add_argument('--column-a', help='Selected column in the first CSV')
    parser.add_argument('--column-b', help='Selected column in the second CSV '
                        '(or first CSV if no second file is supplied)')
    parser.add_argument('--label-a', help='Optional method A plot label')
    parser.add_argument('--label-b', help='Optional method B plot label')
    parser.add_argument('--run-column', default='run_idx',
                        help='Pairing key for files (default: run_idx)')
    parser.add_argument('--output', type=Path,
                        help='Figure filename; omit for an interactive plot')
    return parser


def main(argv=None):
    """Validate arguments and run visual assessment without normality tests."""
    parser = build_parser()
    args = parser.parse_args(argv)
    args.input = args.input.expanduser()
    if args.output:
        args.output = args.output.expanduser()
        matplotlib.use('Agg')
    try:
        if args.input.is_dir():
            parser.error('Use qq_plot_batch.py for batch or campaign folders.')
        if not args.column_a or not args.column_b:
            parser.error('CSV mode requires --column-a and --column-b.')
        if args.output and any(
            args.output.resolve() == path.expanduser().resolve()
            for path in (args.input, args.csv_b) if path is not None
        ):
            parser.error('--output must differ from the input CSV paths.')
        frame_a = read_csv(args.input)
        frame_b = read_csv(args.csv_b) if args.csv_b else None
        difference, dropped = paired_differences(
            frame_a, args.column_a, args.input, frame_b,
            args.column_b, args.csv_b, args.run_column)
        label_a = args.label_a or (
            method_name(args.input) if args.csv_b else args.column_a)
        label_b = args.label_b or (
            method_name(args.csv_b) if args.csv_b else args.column_b)
        context = ''
        if args.csv_b:
            context = (args.column_a if args.column_a == args.column_b else
                       f'{args.column_a} - {args.column_b}')
        describe_differences(difference, label_a, label_b, dropped, context)
        plot_differences(difference, label_a, label_b, args.output, context)
        return 0
    except (ValueError, OSError) as error:
        print(f'Error: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
