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

"""Read a campaign's runs.json and select accepted recordings without ROS."""

import json
import os
from pathlib import Path
import re

RUN_ID = re.compile(r'run_(\d+)_(\d+)')
RUNS_FILE = 'runs.json'
SCHEMA_VERSION = 2
STATUSES = ('running', 'completed', 'failed', 'interrupted')
BAG_STATES = (None, 'pending', 'accepted', 'failed')
ENTRY_KEYS = (
    'adaptation_manager', 'mission_name', 'experiment_launch', 'status',
    'attempt_id', 'bag', 'failure_type', 'failure_reason', 'started_at',
    'finished_at', 'attempts',
)
BAG_PREFIXES = {
    'pending': 'rosbags/pending/',
    'accepted': 'rosbags/',
    'failed': 'rosbags/failed/',
}


def atomic_write_json(path, data):
    """Durably replace a JSON file: temporary file, fsync, then rename."""
    path = Path(path)
    temporary = path.with_name(f'{path.name}.tmp')
    with temporary.open('w', encoding='utf-8') as handle:
        handle.write(json.dumps(data, indent=2) + '\n')
        handle.flush()
        os.fsync(handle.fileno())
    temporary.replace(path)


def load_runs(campaign):
    """Read a campaign's runs.json, rejecting unsupported or incomplete data."""
    path = Path(campaign) / RUNS_FILE
    data = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict) or (
            data.get('schema_version') != SCHEMA_VERSION):
        raise ValueError(f'Unsupported runs schema: {path}')
    if not isinstance(data.get('campaign'), dict) or not isinstance(
            data.get('runs'), dict):
        raise ValueError(f'Invalid runs file: {path}')
    for run_id, entry in data['runs'].items():
        parse_run_id(run_id)
        missing = [key for key in ENTRY_KEYS if key not in entry]
        if missing:
            raise ValueError(
                f'Incomplete run {run_id} in {path}: {", ".join(missing)}')
        if entry['status'] not in STATUSES or entry['bag'] not in BAG_STATES:
            raise ValueError(f'Invalid status or bag for {run_id} in {path}')
    return data


def write_runs(campaign, data):
    """Atomically replace a campaign's runs.json."""
    atomic_write_json(Path(campaign) / RUNS_FILE, data)


def parse_run_id(run_id):
    """Return (experiment index, run index) of a run_<e>_<r> identifier."""
    match = RUN_ID.fullmatch(run_id)
    if not match:
        raise ValueError(f'Invalid run id: {run_id!r}')
    return int(match.group(1)), int(match.group(2))


def bag_name(run_id, entry, attempt_id=None):
    """Return an attempt's bag folder name; old attempts pass their id."""
    return (f'{entry["adaptation_manager"]}_{entry["mission_name"]}_'
            f'{run_id}_{attempt_id or entry["attempt_id"]}')


def bag_dir(campaign, run_id, entry, state=None, attempt_id=None):
    """Return where an attempt's bag lives in the given (or current) state."""
    state = state or entry['bag']
    if state not in BAG_PREFIXES:
        raise ValueError(f'No bag location for state {state!r}: {run_id}')
    return artifact_path(
        campaign, BAG_PREFIXES[state] + bag_name(run_id, entry, attempt_id))


def log_dir(campaign, run_id, entry):
    """Return the current attempt's log folder."""
    return artifact_path(campaign, f'logs/{run_id}/{entry["attempt_id"]}')


def artifact_path(campaign, relative):
    """Resolve a safe campaign-relative artifact path, including symlinks."""
    if not isinstance(relative, str) or not relative:
        raise ValueError(f'Invalid artifact path: {relative!r}')
    path = Path(relative)
    root = Path(campaign).resolve()
    resolved = (root / path).resolve()
    escapes = path.is_absolute() or '..' in path.parts
    if escapes or not resolved.is_relative_to(root):
        raise ValueError(f'Artifact escapes campaign: {relative}')
    if resolved == root:
        raise ValueError('Artifact cannot be the campaign directory')
    return resolved


def accepted_runs(campaign):
    """Return completed runs' accepted recordings and every exclusion."""
    campaign = Path(campaign)
    runs = load_runs(campaign)['runs']
    selected, excluded, seen = [], [], set()
    for run_id in sorted(runs, key=parse_run_id):
        entry = runs[run_id]
        if entry['status'] != 'completed':
            continue
        if entry['bag'] is None:
            excluded.append({'run_id': run_id, 'reason': 'recording_disabled'})
            continue
        if entry['bag'] != 'accepted':
            raise ValueError(f'Completed run without accepted bag: {run_id}')
        bag = bag_dir(campaign, run_id, entry)
        if not (bag / 'metadata.yaml').is_file():
            raise ValueError(f'Missing accepted recording metadata: {bag}')
        seen.add(bag)
        selected.append({
            'run_id': run_id,
            'attempt_id': entry['attempt_id'],
            'adaptation_manager': entry['adaptation_manager'],
            'bag_path': str(bag.relative_to(campaign.resolve())),
        })
    excluded.extend(_unselected_bags(campaign, seen))
    return selected, excluded


def accepted_attempts(campaign):
    """Return {manager: {run_idx: attempt_id}} for every completed run."""
    runs = load_runs(campaign)['runs']
    accepted, experiments = {}, {}
    for run_id in sorted(runs, key=parse_run_id):
        entry = runs[run_id]
        exp_idx, run_idx = parse_run_id(run_id)
        manager = entry['adaptation_manager']
        if experiments.setdefault(manager, exp_idx) != exp_idx:
            raise ValueError(
                f'{campaign}: manager {manager!r} appears in experiments '
                f'{experiments[manager]} and {exp_idx}')
        if entry['status'] == 'completed':
            accepted.setdefault(manager, {})[run_idx] = entry['attempt_id']
    return accepted


def _unselected_bags(campaign, seen):
    exclusions = []
    for metadata in sorted((campaign / 'rosbags').glob('**/metadata.yaml')):
        bag = metadata.parent.resolve()
        if bag not in seen:
            exclusions.append({
                'bag_path': str(metadata.parent.relative_to(campaign)),
                'reason': 'not_selected',
            })
    return exclusions
