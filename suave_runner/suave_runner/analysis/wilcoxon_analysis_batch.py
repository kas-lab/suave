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

"""Orchestrate existing paired Wilcoxon analysis for every campaign in a batch."""

import argparse
from pathlib import Path
import sys

import numpy as np

import pandas as pd

from suave_tools.layout import batch_layout
from suave_tools.layout import RESULTS_DIR


if __package__:
    from . import common
    from . import wilcoxon_analysis
else:
    import common
    import wilcoxon_analysis


HELP_TEXT = """Orchestrate existing paired Wilcoxon analysis for every campaign in a batch.

Run in the sourced SUAVE container with common.py and wilcoxon_analysis.py
alongside:

    python3 wilcoxon_analysis_batch.py /path/to/batch
    python3 wilcoxon_analysis_batch.py /path/to/batch/campaigns/exp1
    python3 wilcoxon_analysis_batch.py /path/to/batch --output /path/to/results

The input is a batch, its campaigns/ directory, its state.json, or one
campaign (a folder with runs.json).

Each campaign's managers come from its runs.json; their raw
<manager>_<mission>.csv files are filtered to each run's accepted attempt by
the run_idx and attempt_id columns. Write analysis results to
<batch>/campaign_results/<experiment>/wilcoxon_analysis/ by default.
Runs must share scenarios within each campaign. Campaigns are never paired
with each other. Both metrics exclude
pairs where either method did not find the pipeline. Holm correction is
applied separately per campaign across all computed ordered-pair tests of
search time (less) and distance inspected (greater).
"""


def validate_frame(frame, path):
    """Require unique integer run IDs, finite metrics, and valid status."""
    columns = ['run_idx', 'pipeline found']
    columns += [metric[0] for metric in wilcoxon_analysis.METRICS]
    missing = [column for column in columns if column not in frame.columns]
    if missing:
        raise ValueError(f'{path}: missing required columns: {missing}')
    if frame.empty:
        raise ValueError(f'{path}: no experimental runs')
    for column in ['run_idx'] + columns[2:]:
        try:
            values = pd.to_numeric(frame[column], errors='raise')
        except (ValueError, TypeError) as error:
            raise ValueError(
                f'{path}: {column!r} contains non-numeric values') from error
        if not np.isfinite(values.to_numpy(dtype=float)).all():
            raise ValueError(
                f'{path}: {column!r} contains missing or non-finite values')
        if column == 'run_idx':
            if (values < 0).any() or (values % 1 != 0).any():
                raise ValueError(
                    f'{path}: run_idx must be nonnegative integers')
            if values.duplicated().any():
                raise ValueError(f'{path}: duplicate run_idx values')
    common.parse_pipeline_found(frame['pipeline found'], path)
    return set(frame['run_idx'])


def campaign_systems(campaign):
    """Load every method's accepted rows and require identical run sets."""
    methods = common.campaign_methods(campaign)
    if len(methods) < 2:
        raise ValueError(f'{campaign}: runs.json needs at least two managers')
    systems = []
    expected_ids = None
    for manager, path in methods:
        frame = wilcoxon_analysis.load_system(manager, path)
        ids = validate_frame(frame, path)
        if expected_ids is None:
            expected_ids = ids
        elif ids != expected_ids:
            raise ValueError(
                f'{path}: run_idx mismatch relative to {methods[0][1].name}; '
                f'missing {sorted(expected_ids - ids)}, '
                f'extra {sorted(ids - expected_ids)}')
        systems.append((manager, frame))
    return systems


def analyze_campaign(campaign, output_root, correction):
    """Run the paired Wilcoxon analysis for one campaign and save it."""
    systems = campaign_systems(campaign)
    destination = output_root / campaign.name / 'wilcoxon_analysis'
    filename = f'{campaign.name}_wilcoxon'
    results = wilcoxon_analysis.analyze(systems, correction)
    results_path = wilcoxon_analysis.save_results(
        results, systems, destination, filename)
    computed = sum(not np.isnan(result['p_raw']) for result in results)
    not_computed = len(results) - computed
    print(f'{campaign.name}: {computed} tests computed, '
          f'{not_computed} not computed; {results_path}', flush=True)
    return results_path


def build_parser():
    """Create command-line options for batch input and output location."""
    parser = argparse.ArgumentParser(
        description=HELP_TEXT,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('input', type=Path,
                        help='Batch, campaigns directory, state.json, or '
                        'campaign')
    parser.add_argument('--output', type=Path,
                        help=f'Output root; defaults to <batch>/{RESULTS_DIR}')
    parser.add_argument('--correction', choices=('holm', 'none'),
                        default='none', help='Multiplicity correction '
                        'within each campaign (default: none)')
    return parser


def main(argv=None):
    """Analyze each campaign independently and report any failures."""
    args = build_parser().parse_args(argv)
    try:
        layout = batch_layout(args.input)
        if not layout.campaigns:
            raise ValueError(f'No campaigns found in {layout.root}')
    except (ValueError, OSError) as error:
        print(f'Error: {error}', file=sys.stderr)
        return 1
    campaigns = layout.campaigns
    output_root = (args.output.expanduser().resolve() if args.output else
                   layout.results_root)
    failures = 0
    for campaign in campaigns:
        try:
            analyze_campaign(campaign, output_root, args.correction)
        except (ValueError, OSError, RuntimeError) as error:
            print(f'Error: {campaign.name}: {error}', file=sys.stderr)
            failures += 1
    print(f'Completed {len(campaigns) - failures} campaigns; '
          f'{failures} failures.')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
