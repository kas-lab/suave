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
"""Export requirement fulfillment data for LaTeX PGFPlots figures."""

import argparse
import json
from pathlib import Path

import numpy as np  # noqa: I100,I201

import plot_requirement_fulfillment as fulfillment_plots  # noqa: I100,I201
from suave_tools.layout import batch_layout  # noqa: I100,I201
from suave_tools.layout import RESULTS_DIR  # noqa: I100,I201


DEFAULT_PRECISION = 6


def parse_args(argv=None):
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description=('Export SUAVE requirement-fulfillment summaries as '
                     'PGFPlots-friendly data tables. Run it in the sourced '
                     'SUAVE container.'))
    parser.add_argument(
        'path',
        help='Batch, campaigns directory, state.json, or campaign folder.')
    parser.add_argument(
        '--output-dir',
        help=('Output root; each campaign writes data and manifest.json to '
              f'<output-dir>/<campaign>/ (default: <batch>/{RESULTS_DIR}/'
              '<campaign>/requirement_fulfillment_tikz).'))
    parser.add_argument(
        '--managers', nargs='+', default=fulfillment_plots.MANAGERS,
        help='Manager identifiers to include.')
    parser.add_argument(
        '--samples', type=int, default=1001,
        help='Number of normalized recording-time samples per run.')
    parser.add_argument(
        '--precision', type=int, default=DEFAULT_PRECISION,
        help='Decimal places for floating-point table values.')
    return parser.parse_args(argv)


def table_filename(requirement_key, manager):
    """Return a stable table filename for one requirement and manager."""
    return f'{requirement_key}_{manager}.dat'


def table_rows(grid, mean, ci, counts):
    """Return finite rows for a PGFPlots table."""
    rows = []
    for time_percent, value, interval, count in zip(grid, mean, ci, counts):
        if not np.isfinite(value) or count <= 0:
            continue
        low = float(np.clip(value - interval, 0.0, 1.0))
        high = float(np.clip(value + interval, 0.0, 1.0))
        rows.append({
            'time_percent': float(time_percent),
            'mean': float(value),
            'ci_low': low,
            'ci_high': high,
            'ci': float(interval),
            'count': int(count),
        })
    return rows


def write_table(path, rows, precision=DEFAULT_PRECISION):
    """Write one whitespace-delimited PGFPlots data table."""
    path.parent.mkdir(parents=True, exist_ok=True)
    float_format = f'{{:.{precision}f}}'
    lines = ['time_percent mean ci_low ci_high ci count']
    for row in rows:
        lines.append(' '.join([
            float_format.format(row['time_percent']),
            float_format.format(row['mean']),
            float_format.format(row['ci_low']),
            float_format.format(row['ci_high']),
            float_format.format(row['ci']),
            str(row['count']),
        ]))
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def export_requirement_tables(requirement_key, runs_by_manager, managers, grid,
                              output_dir, precision):
    """Write all tables for one requirement and return manifest entries."""
    entries = {}
    for manager in managers:
        runs = runs_by_manager.get(manager, [])
        if not runs:
            continue
        mean, ci, counts = fulfillment_plots.mean_and_ci(runs)
        rows = table_rows(grid, mean, ci, counts)
        if not rows:
            continue
        relative_path = Path('data') / table_filename(requirement_key, manager)
        write_table(output_dir / relative_path, rows, precision)
        entries[manager] = {
            'file': relative_path.as_posix(),
            'rows': len(rows),
            'runs': len(runs),
            'label': fulfillment_plots.MANAGER_LABELS.get(manager, manager),
            'color': fulfillment_plots.COLORS.get(manager, 'black'),
        }
    return entries


def build_manifest(campaign, args, selected, excluded, grouped, invalid, grid,
                   output_dir):
    """Build and write all requirement data tables for one campaign."""
    manifest = {
        'campaign': str(campaign),
        'grid': {
            'samples': int(len(grid)),
            'min_percent': float(grid[0]),
            'max_percent': float(grid[-1]),
        },
        'options': vars(args),
        'selected': selected,
        'exclusions': excluded,
        'per_manager_accepted_counts': {},
        'requirements': {},
    }
    for entry in selected:
        manager = entry['adaptation_manager']
        manifest['per_manager_accepted_counts'][manager] = (
            manifest['per_manager_accepted_counts'].get(manager, 0) + 1)
    for requirement_key, spec in fulfillment_plots.REQUIREMENTS.items():
        tables = export_requirement_tables(
            requirement_key, grouped[requirement_key], args.managers, grid,
            output_dir, args.precision)
        manifest['requirements'][requirement_key] = {
            'topic': spec['topic'],
            'label': spec['label'],
            'tables': tables,
            'contributing_runs': {
                manager: len(grouped[requirement_key].get(manager, []))
                for manager in args.managers
            },
            'invalid_samples': invalid[requirement_key],
        }
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / 'manifest.json').write_text(
        json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    return manifest


def export_campaign(campaign, output_dir, args, grid):
    """Export one campaign's requirement-fulfillment tables."""
    selected, excluded = fulfillment_plots.accepted_entries(
        campaign, args.managers)
    print(f'Campaign: {campaign}')
    print(f'Selected accepted bag(s): {len(selected)}')
    records = [fulfillment_plots.read_bag(entry, campaign)
               for entry in selected]
    grouped, invalid = fulfillment_plots.build_runs(records, grid)
    manifest = build_manifest(
        campaign, args, selected, excluded, grouped, invalid, grid,
        output_dir)
    table_count = sum(
        len(requirement['tables'])
        for requirement in manifest['requirements'].values())
    print(f'Wrote {table_count} data table(s) to {output_dir / "data"}')
    print(f'Wrote manifest to {output_dir / "manifest.json"}')


def main(argv=None):
    """Export requirement-fulfillment tables for every campaign."""
    args = parse_args(argv)
    if args.samples < 2:
        raise ValueError('--samples must be at least 2')
    if args.precision < 0:
        raise ValueError('--precision must be non-negative')
    layout = batch_layout(args.path)
    if not layout.campaigns:
        raise ValueError(f'No campaigns found in {layout.root}')
    grid = np.linspace(0.0, 100.0, args.samples)
    for campaign in layout.campaigns:
        if args.output_dir:
            output_dir = Path(args.output_dir).expanduser() / campaign.name
        else:
            output_dir = (layout.results_root / campaign.name /
                          'requirement_fulfillment_tikz')
        export_campaign(campaign, output_dir, args, grid)


if __name__ == '__main__':
    main()
