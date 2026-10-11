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
"""Plot mean requirement fulfillment from accepted SUAVE MCAP outputs."""

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt  # noqa: I100,I201
import numpy as np  # noqa: I100,I201
import yaml  # noqa: I100,I201
from rclpy.serialization import deserialize_message  # noqa: I100,I201
from rosbag2_py import ConverterOptions  # noqa: I100,I201
from rosbag2_py import SequentialReader  # noqa: I100,I201
from rosbag2_py import StorageFilter  # noqa: I100,I201
from rosbag2_py import StorageOptions  # noqa: I100,I201
from std_msgs.msg import Float32  # noqa: I100,I201
from suave_requirements.requirement_monitor import (  # noqa: I100,I201
    fulfillment_topic, REACTION_TIME, REACTION_TIME_TYPES, SEARCH_FOOTPRINT,
    THRUSTER_AVAILABILITY)
from suave_tools.runs import accepted_runs  # noqa: I100,I201


REQUIREMENTS = {
    THRUSTER_AVAILABILITY: {
        'topic': fulfillment_topic(THRUSTER_AVAILABILITY),
        'label': 'Thruster availability',
    },
    SEARCH_FOOTPRINT: {
        'topic': fulfillment_topic(SEARCH_FOOTPRINT),
        'label': 'Search footprint',
    },
    REACTION_TIME: {
        'topic': fulfillment_topic(REACTION_TIME),
        'label': 'Adaptation reaction time',
    },
}
REACTION_TIME_LABELS = {
    'thruster': 'Thruster reaction time',
    'water_visibility': 'Water-visibility reaction time',
    'battery': 'Battery reaction time',
}
REQUIREMENTS.update({
    reaction_type + '_reaction_time': {
        'topic': fulfillment_topic(REACTION_TIME + '/' + reaction_type),
        'label': REACTION_TIME_LABELS[reaction_type],
    }
    for reaction_type in REACTION_TIME_TYPES
})

MANAGERS = ['rebetmc', 'rosa_bt', 'bt', 'metacontrol', 'random', 'none']
MANAGER_LABELS = {
    'rebetmc': 'TacticalRetreat',
    'rosa_bt': 'ROSA-BT',
    'bt': 'BT',
    'metacontrol': 'Metacontrol',
    'random': 'Random',
    'none': 'None',
}
COLORS = {
    'rebetmc': '#0072B2',
    'rosa_bt': '#009E73',
    'bt': '#E69F00',
    'metacontrol': '#CC79A7',
    'random': '#D55E00',
    'none': '#555555',
}
TOPICS = {spec['topic']: key for key, spec in REQUIREMENTS.items()}


def parse_args():
    """Parse command-line arguments."""
    default_bags = (
        '~/suave/results/batches/batch_20261009_075625/campaigns/exp3/'
        'rosbags'
    )
    parser = argparse.ArgumentParser(
        description='Generate one plot per SUAVE requirement fulfillment.')
    parser.add_argument(
        '--bags-dir', default=default_bags,
        help='Directory containing accepted rosbag directories.')
    parser.add_argument(
        '--output-dir', default='requirement_fulfillment_plots',
        help='Directory where figures and coverage.json are written.')
    parser.add_argument(
        '--managers', nargs='+', default=MANAGERS,
        help='Manager identifiers to include.')
    parser.add_argument(
        '--samples', type=int, default=1001,
        help='Number of normalized recording-time samples per run.')
    parser.add_argument(
        '--formats', nargs='+', default=['png', 'pdf'],
        help='Figure formats to write, e.g. png pdf svg.')
    return parser.parse_args()


def metadata_info(bag_dir):
    """Return parsed rosbag metadata information."""
    data = yaml.safe_load(
        (bag_dir / 'metadata.yaml').read_text(encoding='utf-8'))
    return data['rosbag2_bagfile_information']


def accepted_entries(bags_dir, managers):
    """Load the accepted bags of a campaign's completed runs."""
    campaign = bags_dir.parent
    selected, excluded = accepted_runs(campaign)
    selected = [entry for entry in selected
                if entry['adaptation_manager'] in managers]
    return campaign, selected, excluded


def read_bag(entry, campaign):
    """Read all requirement topics from one accepted bag."""
    bag_dir = campaign / entry['bag_path']
    info = metadata_info(bag_dir)
    start_ns = int(info['starting_time']['nanoseconds_since_epoch'])
    duration_ns = int(info['duration']['nanoseconds'])
    if duration_ns <= 0:
        raise ValueError(f'Invalid duration for {bag_dir}')
    series = {key: {'times_ns': [], 'values': []} for key in REQUIREMENTS}
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag_dir), storage_id='mcap'),
                ConverterOptions('', ''))
    # Let the MCAP reader skip chunks of unrelated (e.g. camera) topics.
    reader.set_filter(StorageFilter(topics=list(TOPICS)))
    invalid = {key: 0 for key in REQUIREMENTS}
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        key = TOPICS.get(topic)
        if key is None:
            continue
        value = float(deserialize_message(data, Float32).data)
        if not np.isfinite(value):
            invalid[key] += 1
            continue
        if value < -1.0e-6 or value > 1.0 + 1.0e-6:
            invalid[key] += 1
            continue
        series[key]['times_ns'].append(int(timestamp) - start_ns)
        series[key]['values'].append(float(np.clip(value, 0.0, 1.0)))
    return {
        **entry,
        'bag_dir': str(bag_dir),
        'duration_ns': duration_ns,
        'series': series,
        'invalid_samples': invalid,
    }


def step_resample(times_ns, values, duration_ns, grid_percent):
    """Resample with previous-value hold on normalized recording time."""
    result = np.full(grid_percent.shape, np.nan, dtype=float)
    if not times_ns:
        return result
    order = np.argsort(np.asarray(times_ns), kind='stable')
    times = np.asarray(times_ns, dtype=np.int64)[order]
    vals = np.asarray(values, dtype=float)[order]
    unique_times, indices = np.unique(times, return_index=True)
    last_indices = np.r_[indices[1:] - 1, len(times) - 1]
    times = unique_times
    vals = vals[last_indices]
    grid_ns = np.rint(duration_ns * grid_percent / 100.0).astype(np.int64)
    positions = np.searchsorted(times, grid_ns, side='right') - 1
    valid = positions >= 0
    result[valid] = vals[positions[valid]]
    return result


def build_runs(records, grid_percent):
    """Return resampled runs grouped by requirement and manager."""
    grouped = {key: {manager: [] for manager in MANAGERS}
               for key in REQUIREMENTS}
    invalid = {key: {} for key in REQUIREMENTS}
    for record in records:
        manager = record['adaptation_manager']
        for key in REQUIREMENTS:
            run = step_resample(
                record['series'][key]['times_ns'],
                record['series'][key]['values'],
                record['duration_ns'], grid_percent)
            if np.any(np.isfinite(run)):
                grouped[key].setdefault(manager, []).append(run)
            invalid[key][record['run_id']] = record['invalid_samples'][key]
    return grouped, invalid


def mean_and_ci(runs):
    """Return finite-sample mean, confidence interval and counts."""
    array = np.vstack(runs)
    finite = np.isfinite(array)
    counts = np.sum(finite, axis=0)
    mean = np.full(array.shape[1], np.nan)
    ci = np.zeros(array.shape[1])
    positive = counts > 0
    mean[positive] = np.nanmean(array[:, positive], axis=0)
    multi = counts > 1
    if np.any(multi):
        std = np.nanstd(array[:, multi], axis=0, ddof=1)
        ci[multi] = 1.96 * std / np.sqrt(counts[multi])
    return mean, ci, counts


def prettify_axes(axis, title):
    """Apply a consistent publication-style look to one axis."""
    axis.set_title(title, fontsize=14, weight='bold')
    axis.set_xlabel('Normalized recording time (%)')
    axis.set_ylabel('Mean requirement fulfillment')
    axis.set_xlim(0.0, 100.0)
    axis.set_ylim(-0.03, 1.03)
    axis.grid(True, which='major', color='#dddddd', linewidth=0.8)
    axis.spines['top'].set_visible(False)
    axis.spines['right'].set_visible(False)


def plot_no_data_requirement(key, spec, output_dir, formats):
    """Write a placeholder figure for a requirement with no messages."""
    fig, axis = plt.subplots(figsize=(8.0, 4.8), constrained_layout=True)
    prettify_axes(axis, spec['label'])
    axis.text(
        0.5, 0.5,
        f'No selected accepted bags recorded\n{spec["topic"]}',
        transform=axis.transAxes,
        ha='center', va='center', fontsize=12, color='#555555')
    save_figure(fig, output_dir, key, formats)


def save_figure(fig, output_dir, key, formats):
    """Save one figure in every requested format."""
    output_dir.mkdir(parents=True, exist_ok=True)
    for suffix in formats:
        fig.savefig(output_dir / f'{key}.{suffix}', dpi=300)
    plt.close(fig)


def plot_requirement(key, spec, runs_by_manager, managers, grid, output_dir,
                     formats, report, accepted_counts):
    """Write one manager-comparison figure for a single requirement."""
    fig, axis = plt.subplots(figsize=(8.0, 4.8), constrained_layout=True)
    missing, unaccepted = [], []
    for manager in managers:
        runs = runs_by_manager.get(manager, [])
        if not runs:
            label = MANAGER_LABELS.get(manager, manager)
            if not accepted_counts.get(manager):
                unaccepted.append(label)
            else:
                missing.append(label)
            continue
        mean, ci, counts = mean_and_ci(runs)
        label = f'{MANAGER_LABELS.get(manager, manager)} (n={len(runs)})'
        axis.plot(grid, mean, color=COLORS.get(manager), linewidth=2.2,
                  label=label)
        axis.fill_between(
            grid, np.clip(mean - ci, 0.0, 1.0),
            np.clip(mean + ci, 0.0, 1.0), color=COLORS.get(manager),
            alpha=0.14, linewidth=0.0)
        report['per_grid_counts'][manager] = counts.astype(int).tolist()
    report['no_observations'] = missing
    report['no_accepted_runs'] = unaccepted
    prettify_axes(axis, spec['label'])
    axis.legend(frameon=False, ncol=2, fontsize=9, loc='lower left')
    caption = ('Curves show cross-run means over selected accepted '
               'recordings; bands are approximate 95% confidence intervals.')
    if missing:
        caption += f'\nNo observations (not plotted): {", ".join(missing)}.'
    if unaccepted:
        caption += f'\nNo accepted runs (not plotted): {", ".join(unaccepted)}.'
    # supxlabel takes part in constrained layout, unlike fig.text.
    fig.supxlabel(caption, fontsize=8, color='#555555')
    save_figure(fig, output_dir, key, formats)


def write_coverage(output_dir, coverage):
    """Write coverage/provenance report as JSON."""
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / 'coverage.json').write_text(
        json.dumps(coverage, indent=2) + '\n', encoding='utf-8')


def main():
    """Generate requirement-fulfillment figures."""
    args = parse_args()
    if args.samples < 2:
        raise ValueError('--samples must be at least 2')
    bags_dir = Path(args.bags_dir).expanduser()
    output_dir = Path(args.output_dir).expanduser()
    grid = np.linspace(0.0, 100.0, args.samples)
    campaign, selected, excluded = accepted_entries(bags_dir, args.managers)
    print(f'Reading accepted bags from: {bags_dir}')
    print(f'Campaign: {campaign}')
    print(f'Selected accepted bag(s): {len(selected)}')
    records = [read_bag(entry, campaign) for entry in selected]
    grouped, invalid = build_runs(records, grid)
    coverage = {
        'campaign': str(campaign),
        'bags_dir': str(bags_dir),
        'options': vars(args),
        'selected': selected,
        'exclusions': excluded,
        'per_manager_accepted_counts': {},
        'requirements': {},
    }
    for entry in selected:
        manager = entry['adaptation_manager']
        coverage['per_manager_accepted_counts'][manager] = (
            coverage['per_manager_accepted_counts'].get(manager, 0) + 1)
    for key, spec in REQUIREMENTS.items():
        requirement_report = {
            'topic': spec['topic'],
            'contributing_runs': {
                manager: len(grouped[key].get(manager, []))
                for manager in args.managers
            },
            'invalid_samples': invalid[key],
            'per_grid_counts': {},
        }
        coverage['requirements'][key] = requirement_report
        if not any(requirement_report['contributing_runs'].values()):
            plot_no_data_requirement(key, spec, output_dir, args.formats)
        else:
            plot_requirement(
                key, spec, grouped[key], args.managers, grid, output_dir,
                args.formats, requirement_report,
                coverage['per_manager_accepted_counts'])
        print(f'Wrote {key} figure to {output_dir}')
    write_coverage(output_dir, coverage)
    print(f'Wrote coverage report to {output_dir / "coverage.json"}')


if __name__ == '__main__':
    main()
