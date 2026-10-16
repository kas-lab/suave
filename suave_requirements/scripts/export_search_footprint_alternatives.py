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
"""Export alternative search-footprint summaries for PGFPlots."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np  # noqa: I100,I201

import plot_requirement_fulfillment as fulfillment_plots  # noqa: I100,I201
from suave_tools.layout import batch_layout  # noqa: I100,I201
from suave_tools.runs import parse_run_id  # noqa: I100,I201


SEARCH_KEY = 'search_footprint'
DEFAULT_PRECISION = 6


def parse_args(argv=None):
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description=('Export censored and accumulated search-footprint data '
                     'for LaTeX/PGFPlots.'))
    parser.add_argument(
        'path',
        help='Batch, campaigns directory, state.json, or campaign folder.')
    parser.add_argument(
        '--output-dir', default='latex_sandbox/generated_extras',
        help='Output root; each campaign writes to <output-dir>/<campaign>/.')
    parser.add_argument(
        '--managers', nargs='+', default=fulfillment_plots.MANAGERS,
        help='Manager identifiers to include.')
    parser.add_argument(
        '--samples', type=int, default=301,
        help='Number of normalized recording-time samples per run.')
    parser.add_argument(
        '--precision', type=int, default=DEFAULT_PRECISION,
        help='Decimal places for floating-point table values.')
    return parser.parse_args(argv)


def manager_csv(campaign, manager):
    """Return the conventional metrics CSV for one manager."""
    return Path(campaign) / f'{manager}_suave.csv'


def bool_value(value):
    """Return whether a CSV boolean field is true."""
    return str(value).strip().lower() in ('1', 'true', 'yes')


def load_detection_times(campaign, managers):
    """Return per-run pipeline-detection times from campaign CSV files."""
    detections = {}
    for manager in managers:
        path = manager_csv(campaign, manager)
        if not path.is_file():
            continue
        manager_detections = {}
        with path.open(newline='', encoding='utf-8') as handle:
            for row in csv.DictReader(handle):
                if not bool_value(row.get('pipeline found')):
                    continue
                run_idx = int(row['run_idx'])
                manager_detections[run_idx] = {
                    'attempt_id': row.get('attempt_id', ''),
                    'search_time_s': float(
                        row['time searching pipeline (s)']),
                    'mission_duration_s': float(row['mission duration (s)']),
                }
        detections[manager] = manager_detections
    return detections


def mean_ci_scalar(values):
    """Return mean and approximate 95% confidence interval for scalars."""
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return np.nan, 0.0
    if values.size == 1:
        return float(values[0]), 0.0
    return float(np.mean(values)), float(
        1.96 * np.std(values, ddof=1) / np.sqrt(values.size))


def finite_table_rows(grid, mean, ci, counts):
    """Return finite rows for a PGFPlots data table."""
    rows = []
    for time_percent, value, interval, count in zip(grid, mean, ci, counts):
        if not np.isfinite(value) or count <= 0:
            continue
        rows.append({
            'time_percent': float(time_percent),
            'mean': float(value),
            'ci_low': float(np.clip(value - interval, 0.0, 1.0)),
            'ci_high': float(np.clip(value + interval, 0.0, 1.0)),
            'ci': float(interval),
            'count': int(count),
        })
    return rows


def write_rows(path, header, rows, precision=DEFAULT_PRECISION):
    """Write rows as a whitespace-delimited PGFPlots table."""
    path.parent.mkdir(parents=True, exist_ok=True)
    float_format = f'{{:.{precision}f}}'
    lines = [' '.join(header)]
    for row in rows:
        values = []
        for key in header:
            value = row[key]
            if isinstance(value, float):
                values.append(float_format.format(value))
            else:
                values.append(str(value))
        lines.append(' '.join(values))
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def censored_run(record, detection, grid):
    """Return one run censored after its first pipeline detection."""
    run = fulfillment_plots.step_resample(
        record['series'][SEARCH_KEY]['times_ns'],
        record['series'][SEARCH_KEY]['values'],
        record['duration_ns'], grid)
    detection_percent = 100.0 * (
        detection['search_time_s'] / detection['mission_duration_s'])
    run[grid > detection_percent] = np.nan
    return run, detection_percent


def accumulated_values(run, grid, detection_percent, detection_time_s):
    """Return accumulated and active-average fulfillment before detection."""
    active = (grid <= detection_percent) & np.isfinite(run)
    if np.sum(active) < 2 or detection_percent <= 0.0:
        return np.nan, np.nan
    active_percent = grid[active]
    active_time = active_percent * detection_time_s / detection_percent
    auc = float(np.trapz(run[active], active_time))
    active_average = auc / detection_time_s
    return auc, active_average


def build_search_records(campaign, args, grid):
    """Read bags and return censored runs plus accumulated per-run values."""
    selected, excluded = fulfillment_plots.accepted_entries(
        campaign, args.managers)
    detections = load_detection_times(campaign, args.managers)
    by_manager = {manager: [] for manager in args.managers}
    accumulated = {manager: [] for manager in args.managers}
    skipped = []
    for entry in selected:
        manager = entry['adaptation_manager']
        _, run_idx = parse_run_id(entry['run_id'])
        detection = detections.get(manager, {}).get(run_idx)
        if not detection:
            skipped.append({'run_id': entry['run_id'], 'reason': 'not_found'})
            continue
        if detection['attempt_id'] and (
                detection['attempt_id'] != entry['attempt_id']):
            skipped.append({
                'run_id': entry['run_id'],
                'reason': 'attempt_id_mismatch',
            })
            continue
        record = fulfillment_plots.read_bag(entry, campaign)
        run, detection_percent = censored_run(record, detection, grid)
        if not np.any(np.isfinite(run)):
            skipped.append({
                'run_id': entry['run_id'],
                'reason': 'no_samples',
            })
            continue
        auc, active_average = accumulated_values(
            run, grid, detection_percent, detection['search_time_s'])
        by_manager.setdefault(manager, []).append(run)
        accumulated.setdefault(manager, []).append({
            'run_id': entry['run_id'],
            'attempt_id': entry['attempt_id'],
            'detection_time_s': detection['search_time_s'],
            'detection_percent': detection_percent,
            'auc_fulfillment_s': auc,
            'active_average': active_average,
        })
    return selected, excluded, skipped, by_manager, accumulated


def write_censored_tables(output_dir, by_manager, managers, grid, precision):
    """Write censored time-series tables for all managers."""
    tables = {}
    for manager in managers:
        runs = by_manager.get(manager, [])
        if not runs:
            continue
        mean, ci, counts = fulfillment_plots.mean_and_ci(runs)
        rows = finite_table_rows(grid, mean, ci, counts)
        path = Path('data') / f'search_footprint_censored_{manager}.dat'
        write_rows(output_dir / path, [
            'time_percent', 'mean', 'ci_low', 'ci_high', 'ci', 'count',
        ], rows, precision)
        tables[manager] = {'file': path.as_posix(), 'rows': len(rows)}
    return tables


def write_accumulated_tables(output_dir, accumulated, managers, precision):
    """Write per-run and per-manager accumulated search-footprint tables."""
    summary_rows = []
    per_run_rows = []
    for index, manager in enumerate(managers):
        rows = accumulated.get(manager, [])
        finite_rows = [row for row in rows
                       if np.isfinite(row['auc_fulfillment_s'])]
        if not finite_rows:
            continue
        auc_values = [row['auc_fulfillment_s'] for row in finite_rows]
        avg_values = [row['active_average'] for row in finite_rows]
        detection_values = [row['detection_percent'] for row in finite_rows]
        auc_mean, auc_ci = mean_ci_scalar(auc_values)
        avg_mean, avg_ci = mean_ci_scalar(avg_values)
        detection_mean, detection_ci = mean_ci_scalar(detection_values)
        summary_rows.append({
            'manager_index': index,
            'manager': manager,
            'n': len(finite_rows),
            'auc_mean': auc_mean,
            'auc_ci_low': auc_mean - auc_ci,
            'auc_ci_high': auc_mean + auc_ci,
            'active_average_mean': avg_mean,
            'active_average_ci_low': max(0.0, avg_mean - avg_ci),
            'active_average_ci_high': min(1.0, avg_mean + avg_ci),
            'detection_percent_mean': detection_mean,
            'detection_percent_ci_low': detection_mean - detection_ci,
            'detection_percent_ci_high': detection_mean + detection_ci,
        })
        for row in finite_rows:
            per_run_rows.append({
                'manager_index': index,
                'manager': manager,
                **row,
            })
    write_rows(output_dir / 'data/search_footprint_accumulated_summary.dat', [
        'manager_index', 'manager', 'n', 'auc_mean', 'auc_ci_low',
        'auc_ci_high', 'active_average_mean', 'active_average_ci_low',
        'active_average_ci_high', 'detection_percent_mean',
        'detection_percent_ci_low', 'detection_percent_ci_high',
    ], summary_rows, precision)
    write_rows(output_dir / 'data/search_footprint_accumulated_runs.dat', [
        'manager_index', 'manager', 'run_id', 'attempt_id',
        'detection_time_s', 'detection_percent', 'auc_fulfillment_s',
        'active_average',
    ], per_run_rows, precision)
    return {
        'summary': 'data/search_footprint_accumulated_summary.dat',
        'runs': 'data/search_footprint_accumulated_runs.dat',
    }


def export_campaign(campaign, output_dir, args, grid):
    """Export one campaign's alternative search-footprint data."""
    print(f'Campaign: {campaign}')
    (
        selected, excluded, skipped, by_manager, accumulated,
    ) = build_search_records(campaign, args, grid)
    censored_tables = write_censored_tables(
        output_dir, by_manager, args.managers, grid, args.precision)
    accumulated_tables = write_accumulated_tables(
        output_dir, accumulated, args.managers, args.precision)
    manifest = {
        'campaign': str(campaign),
        'selected': selected,
        'exclusions': excluded,
        'skipped': skipped,
        'censored_tables': censored_tables,
        'accumulated_tables': accumulated_tables,
        'notes': [
            'Censored curves omit each run after its own first pipeline '
            'detection time.',
            'auc_fulfillment_s is the time integral of fulfillment while '
            'actively searching.',
            'active_average is auc_fulfillment_s divided by detection time.',
        ],
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / 'search_footprint_extras_manifest.json').write_text(
        json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    print(f'Wrote {len(censored_tables)} censored table(s) to '
          f'{output_dir / "data"}')
    print(f'Wrote accumulated tables to {output_dir / "data"}')


def main(argv=None):
    """Export search-footprint alternatives for every campaign."""
    args = parse_args(argv)
    if args.samples < 2:
        raise ValueError('--samples must be at least 2')
    layout = batch_layout(args.path)
    if not layout.campaigns:
        raise ValueError(f'No campaigns found in {layout.root}')
    grid = np.linspace(0.0, 100.0, args.samples)
    for campaign in layout.campaigns:
        output_dir = Path(args.output_dir).expanduser() / campaign.name
        export_campaign(campaign, output_dir, args, grid)


if __name__ == '__main__':
    main()
