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
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory

import numpy as np

import pandas as pd

import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter

import yaml


if __package__:
    from . import sort_results
    from . import wilcoxon_analysis
else:
    import sort_results
    import wilcoxon_analysis


HELP_TEXT = """Orchestrate existing paired Wilcoxon analysis for every campaign in a batch.

Run in the sourced SUAVE container with sort_results.py and
wilcoxon_analysis.py alongside:

    python3 wilcoxon_analysis_batch.py /path/to/batch
    python3 wilcoxon_analysis_batch.py /path/to/batch --output /path/to/results

Sort raw campaign CSVs using their original completion markers and runner
configs from state.json (or --config-dir). Save verified run_idx-tagged
copies in <batch>/campaigns/<experiment>/sorted/. Write analysis results to
<batch>/campaings_results/<experiment>/wilcoxon_analysis/ by default.
Runs must share scenarios within each campaign. Campaigns are never paired
with each other. Both metrics exclude
pairs where either method did not find the pipeline. Holm correction is
applied separately per campaign across all computed ordered-pair tests of
search time (less) and distance inspected (greater).
"""


def find_campaigns(batch_root):
    """Find experiment folders inside the batch's campaigns folder."""
    campaigns_root = batch_root / 'campaigns'
    if not campaigns_root.is_dir():
        raise ValueError(
            f'Campaigns directory does not exist: {campaigns_root}')
    campaigns = sorted(p for p in campaigns_root.iterdir() if p.is_dir())
    if not campaigns:
        raise ValueError(f'No experiment folders in {campaigns_root}')
    return campaigns


def validate_csv(path):
    """Read a CSV and validate its run IDs, metrics, and detection status."""
    try:
        frame = pd.read_csv(path)
    except (OSError, UnicodeError, pd.errors.ParserError,
            pd.errors.EmptyDataError) as error:
        raise ValueError(f'Cannot read {path}: {error}') from error
    return validate_frame(frame, path)


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
    statuses = frame['pipeline found'].astype(str).str.strip().str.lower()
    if not statuses.isin(['true', 'false']).all():
        raise ValueError(
            f'{path}: pipeline found must contain only true/false values')
    return set(frame['run_idx'])


def campaign_manifest(campaign, entries=None):
    """Validate all sorted method files and require identical run-key sets."""
    if entries is not None:
        expected_ids = None
        for entry in entries:
            ids = validate_csv(entry['data_file'])
            if expected_ids is not None and ids != expected_ids:
                raise ValueError(
                    f'{campaign}: run_idx mismatch between configured methods')
            expected_ids = ids
        return entries
    paths = sorted((campaign / 'sorted').glob('*_sorted.csv'))
    if len(paths) < 2:
        raise ValueError(
            f'{campaign}/sorted needs at least two *_sorted.csv files')
    entries = []
    labels = set()
    expected_ids = None
    for path in paths:
        label = path.stem
        for suffix in ('_sorted', '_suave_extended', '_suave'):
            if label.endswith(suffix):
                label = label[:-len(suffix)]
        if not label or label in labels:
            raise ValueError(
                f'{path}: empty or duplicate method label {label!r}')
        labels.add(label)
        ids = validate_csv(path)
        if expected_ids is None:
            expected_ids = ids
        elif ids != expected_ids:
            raise ValueError(
                f'{path}: run_idx mismatch relative to {paths[0].name}; '
                f'missing {sorted(expected_ids - ids)}, '
                f'extra {sorted(ids - expected_ids)}')
        entries.append({'managing_system': label, 'data_file': str(path)})
    return entries


def resolve_config(campaign, config_dir=None):
    """Locate the recorded runner config or an explicit replacement."""
    state_file = campaign.parent.parent / 'state.json'
    recorded = None
    if state_file.is_file():
        state = json.loads(state_file.read_text())
        if not isinstance(state, dict) or not isinstance(
                state.get('campaigns'), dict):
            raise ValueError(f'{state_file}: missing campaigns mapping')
        matches = []
        for name, entry in state['campaigns'].items():
            if not isinstance(entry, dict):
                raise ValueError(
                    f'{state_file}: invalid campaign entry {name}')
            result_path = Path(entry.get('result_path') or '')
            names = (campaign.name,
                     f'{campaign.name}_suave_runner.launch.py')
            if result_path.name == campaign.name or name in names:
                matches.append(entry)
        if len(matches) > 1:
            raise ValueError(
                f'{state_file}: ambiguous entry for {campaign.name}')
        if matches and matches[0].get('config_file'):
            recorded = Path(matches[0]['config_file']).expanduser()
    if config_dir is not None:
        name = (recorded.name if recorded else
                f'{campaign.name}_runner_config.yml')
        config = config_dir.expanduser() / name
    elif recorded is not None:
        config = (recorded if recorded.is_absolute() else
                  state_file.parent / recorded)
    else:
        raise ValueError(
            f'{campaign}: no runner config recorded in state.json; '
            'provide --config-dir with matching campaign runner configs')
    if not config.is_file():
        raise ValueError(
            f'Runner config does not exist: {config}; use --config-dir '
            'if the original campaign configs have moved')
    return config


def sorting_manifest(campaign, config):
    """Derive source filenames and marker indices from the experiments list."""
    try:
        parameters = yaml.safe_load(config.read_text())[
            '/suave_runner_node']['ros__parameters']
        experiments = parameters['experiments']
        if not isinstance(experiments, list) or len(experiments) < 2:
            raise ValueError('experiments must list at least two methods')
        entries = []
        labels = set()
        for index, value in enumerate(experiments):
            experiment = json.loads(value) if isinstance(value, str) else value
            label = experiment['adaptation_manager']
            mission = experiment['mission_name']
            for part in (label, mission):
                if (not isinstance(part, str) or not part or
                        Path(part).name != part or part in ('.', '..')):
                    raise ValueError('invalid method or mission name')
            if label in labels:
                raise ValueError(f'duplicate method {label!r}')
            labels.add(label)
            count = experiment.get('num_runs', 1)
            if (isinstance(count, bool) or not isinstance(count, int) or
                    count < 1):
                raise ValueError('num_runs must be a positive integer')
            entries.append({
                'managing_system': label,
                'data_file': campaign / f'{label}_{mission}.csv',
                'exp_idx': index, 'num_runs': count,
            })
        return entries
    except (KeyError, TypeError, ValueError, yaml.YAMLError) as error:
        raise ValueError(f'{config}: invalid runner configuration: {error}') \
            from error


def sort_campaign(campaign, config_dir=None):
    """Reconstruct and validate every method before updating sorted outputs."""
    config = resolve_config(campaign, config_dir)
    inputs = sorting_manifest(campaign, config)
    frames = []
    expected_ids = None
    for entry in inputs:
        path = entry['data_file']
        frame = sort_results.reconstruct_run_index(
            path, campaign, entry['exp_idx'])
        if len(frame) != entry['num_runs']:
            raise ValueError(
                f'{path}: {len(frame)} runs, but config declares '
                f'{entry["num_runs"]}')
        ids = validate_frame(frame, path)
        if expected_ids is not None and ids != expected_ids:
            raise ValueError(f'{campaign}: run_idx mismatch between methods')
        expected_ids = ids
        frames.append(frame)
    entries = []
    destination = campaign / 'sorted'
    for entry in inputs:
        output = destination / f'{entry["data_file"].stem}_sorted.csv'
        if output.is_symlink():
            raise ValueError(f'Refusing to overwrite sorted symlink: {output}')
        entries.append({'managing_system': entry['managing_system'],
                        'data_file': str(output)})
    destination.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(prefix='.sorting-', dir=destination) as temporary:
        for entry, frame in zip(entries, frames):
            output = Path(entry['data_file'])
            unchanged = False
            if output.is_file():
                try:
                    pd.testing.assert_frame_equal(
                        pd.read_csv(output), frame, check_dtype=False,
                        check_exact=False, rtol=1e-12, atol=1e-12)
                    unchanged = True
                except (AssertionError, ValueError, pd.errors.ParserError):
                    pass
            if not unchanged:
                staged = Path(temporary) / output.name
                frame.to_csv(staged, index=False)
                staged.replace(output)
            action = 'Verified' if unchanged else 'Sorted'
            print(f'{action}: {output} ({len(frame)} runs)', flush=True)
    return entries


def analyze_campaign(campaign, output_root, correction, config_dir=None):
    """Invoke the existing node and preserve its statistical outputs."""
    entries = campaign_manifest(campaign, sort_campaign(campaign, config_dir))
    destination = output_root / campaign.name / 'wilcoxon_analysis'
    filename = f'{campaign.name}_wilcoxon'
    context = Context()
    node = None
    rclpy.init(args=[], context=context)
    try:
        node = wilcoxon_analysis.WilcoxonAnalysis(
            context=context, use_global_arguments=False,
            parameter_overrides=[
                Parameter('result_path', value=str(destination)),
                Parameter('filename', value=filename),
                Parameter('correction', value=correction),
                Parameter('data_files',
                          value=[json.dumps(e) for e in entries]),
            ])
        # Run directly so analysis exceptions propagate to the batch driver.
        node.perform_analysis()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown(context=context)
    results_path = destination / f'{filename}_results.csv'
    results = pd.read_csv(results_path)
    computed = int(results['p_raw'].notna().sum())
    not_computed = len(results) - computed
    print(f'{campaign.name}: {computed} tests computed, '
          f'{not_computed} not computed; {results_path}', flush=True)
    return results_path


def build_parser():
    """Create command-line options for batch input and output location."""
    parser = argparse.ArgumentParser(
        description=HELP_TEXT,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('input', type=Path, help='Batch root with campaigns/')
    parser.add_argument('--config-dir', type=Path,
                        help='Directory of original campaign runner configs; '
                        'overrides config paths recorded in state.json')
    parser.add_argument('--output', type=Path,
                        help='Output root; defaults to '
                        '<batch>/campaings_results')
    parser.add_argument('--correction', choices=('holm', 'none'),
                        default='none', help='Multiplicity correction '
                        'within each campaign (default: none)')
    return parser


def main(argv=None):
    """Analyze each campaign independently and report any failures."""
    args = build_parser().parse_args(argv)
    batch_root = args.input.expanduser().resolve()
    output_root = (args.output.expanduser().resolve() if args.output else
                   batch_root / 'campaings_results')
    try:
        campaigns = find_campaigns(batch_root)
    except (ValueError, OSError) as error:
        print(f'Error: {error}', file=sys.stderr)
        return 1
    failures = 0
    for campaign in campaigns:
        try:
            analyze_campaign(campaign, output_root, args.correction,
                             args.config_dir)
        except (ValueError, OSError, RuntimeError) as error:
            print(f'Error: {campaign.name}: {error}', file=sys.stderr)
            failures += 1
    print(f'Completed {len(campaigns) - failures} campaigns; '
          f'{failures} failures.')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
