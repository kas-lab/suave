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

"""Save paired-difference Q-Q plots for every campaign in a batch."""

import argparse
from functools import lru_cache
from itertools import combinations
from pathlib import Path
import re
import sys

import matplotlib

from suave_tools.layout import batch_layout
from suave_tools.layout import RESULTS_DIR


if __package__:
    from . import common
    from . import qq_plot
else:
    import common
    import qq_plot


HELP_TEXT = r"""Save paired-difference Q-Q plots for every campaign in a batch.

This orchestrator reuses the CSV validation and plotting in qq_plot.py.
Keep both scripts together. No ROS imports are required.

    python3 qq_plot_batch.py /path/to/batch
    python3 qq_plot_batch.py /path/to/batch --output /path/to/plots
    python3 qq_plot_batch.py /path/to/batch --format png \
        --metrics 'time searching pipeline (s)' 'distance inspected (m)'

Reads each campaign's raw <manager>_<mission>.csv files (managers from its
runs.json), keeps each run's accepted attempt, and saves PNGs under
<batch>/campaign_results/<experiment>/q-q-plots/ by default. The input
is a batch, its campaigns/ directory, its state.json, or one campaign.
All unordered method pairs are compared within each campaign using matched
run_idx IDs.
The default metrics are search time and distance inspected. This script
performs no formal normality test and makes no normality classification.
"""


DEFAULT_METRICS = (
    'time searching pipeline (s)',
    'distance inspected (m)',
)


def filename_component(value):
    """Make a readable, filesystem-safe output name component."""
    return re.sub(r'[^A-Za-z0-9_.-]+', '_', value).strip('._') or 'metric'


def run_batch(args):
    """Plot each unordered method pair per metric within each campaign."""
    metrics = args.metrics or DEFAULT_METRICS
    failures = 0
    completed = 0
    layout = batch_layout(args.input)
    if not layout.campaigns:
        raise ValueError(f'No campaigns found in {layout.root}')
    campaigns = layout.campaigns
    output_root = (args.output.expanduser() if args.output else
                   layout.results_root)
    print(f'Output directory: {output_root}')
    for campaign in campaigns:
        try:
            methods = common.campaign_methods(campaign)
        except (ValueError, OSError) as error:
            print(f'Error: {error}', file=sys.stderr)
            failures += 1
            continue
        if len(methods) < 2:
            print(f'Error: {campaign} needs at least two managers in '
                  'runs.json.', file=sys.stderr)
            failures += 1
            continue
        # Read each file once; failed reads are not cached, so every pair
        # touching a broken file still reports its own failure.
        read_csv = lru_cache(maxsize=None)(common.accepted_rows)
        for pair_number, ((label_a, file_a), (label_b, file_b)) in enumerate(
                combinations(methods, 2)):
            for metric_number, metric in enumerate(metrics):
                context = f'{campaign.name} | {metric}'
                try:
                    difference, dropped = qq_plot.paired_differences(
                        read_csv(file_a, label_a), metric, file_a,
                        read_csv(file_b, label_b), metric, file_b,
                        args.run_column)
                    # Indices prevent collisions after sanitizing names.
                    name = (
                        f'{pair_number:03d}_{metric_number:02d}_'
                        f'{filename_component(label_a)}_minus_'
                        f'{filename_component(label_b)}_'
                        f'{filename_component(metric)}.{args.format}')
                    output = (output_root / campaign.name / 'q-q-plots' / name)
                    qq_plot.describe_differences(
                        difference, label_a, label_b, dropped, context)
                    qq_plot.plot_differences(
                        difference, label_a, label_b, output, context)
                    completed += 1
                except (ValueError, OSError) as error:
                    print(f'Error: {context}, {file_a.name} - '
                          f'{file_b.name}: {error}', file=sys.stderr)
                    failures += 1
    print(f'\nCompleted {completed} plots; {failures} failures.')
    return 1 if failures else 0


def build_parser():
    """Create arguments for batch orchestration and saved figures."""
    parser = argparse.ArgumentParser(
        description=HELP_TEXT,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('input', type=Path,
                        help='Batch, campaigns directory, state.json, or '
                        'campaign')
    parser.add_argument('--metrics', nargs='+', help='Metric columns; '
                        'defaults to search time and distance inspected')
    parser.add_argument('--run-column', default='run_idx',
                        help='Pairing key for files (default: run_idx)')
    parser.add_argument('--output', type=Path,
                        help=f'Output root; defaults to <batch>/{RESULTS_DIR}')
    parser.add_argument('--format', choices=('pdf', 'png', 'svg'),
                        default='png', help='Figure format (default: png)')
    return parser


def main(argv=None):
    """Save batch plots and return a nonzero status on any failure."""
    args = build_parser().parse_args(argv)
    matplotlib.use('Agg')
    try:
        return run_batch(args)
    except (ValueError, OSError) as error:
        print(f'Error: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
