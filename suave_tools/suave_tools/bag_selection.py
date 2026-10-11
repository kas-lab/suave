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

"""Select traceable accepted recordings without ROS dependencies."""

import json
from pathlib import Path
import re

RUN_ID = re.compile(r'run_\d+_\d+')
SCHEMA_VERSION = 1
STATUS_KEYS = (
    'run_id', 'attempt_id', 'adaptation_manager', 'status', 'success',
    'execution_status', 'finalization_state', 'recording_enabled',
    'log_dir', 'bag', 'attempts', 'started_epoch', 'finished_epoch',
)


def load_status(path):
    """Read a status sidecar, rejecting unsupported or incomplete records."""
    path = Path(path)
    record = json.loads(path.read_text())
    if not isinstance(record, dict) or (
            record.get('schema_version') != SCHEMA_VERSION):
        raise ValueError(f'Unsupported status schema: {path}')
    missing = [key for key in STATUS_KEYS if key not in record]
    if missing:
        raise ValueError(f'Incomplete status {path}: {", ".join(missing)}')
    if record['run_id'] != path.name.removesuffix('.status.json'):
        raise ValueError(f'Mismatched status: {path}')
    return record


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


def accepted_entry(record):
    """Validate a checkpointed status and return its accepted bag."""
    if not (record['status'] == 'completed' and record['success'] is True and
            record['execution_status'] == 'completed' and
            record['finalization_state'] == 'finished'):
        raise ValueError(f'Conflicting checkpoint/status: {record["run_id"]}')
    if not record['recording_enabled']:
        return None
    if record['bag']['state'] != 'accepted':
        raise ValueError(f'Unaccepted checkpointed bag: {record["run_id"]}')
    if record['bag']['path'] != record['bag']['accepted_path']:
        raise ValueError('Accepted bag path disagrees with ownership')
    return {
        'run_id': record['run_id'],
        'attempt_id': record['attempt_id'],
        'adaptation_manager': record['adaptation_manager'],
        'bag_path': record['bag']['path'],
    }


def select_accepted_bags(campaign):
    """Return checkpointed accepted recordings and every exclusion."""
    campaign = Path(campaign)
    selected, excluded, seen = [], [], set()
    for marker in sorted(campaign.glob('run_*_*.done')):
        run_id = marker.stem
        if not RUN_ID.fullmatch(run_id):
            raise ValueError(f'Invalid checkpoint: {run_id}')
        path = campaign / f'{run_id}.status.json'
        if not path.is_file():
            raise ValueError(f'Checkpoint without status: {run_id}')
        entry = accepted_entry(load_status(path))
        if entry is None:
            excluded.append({'run_id': run_id, 'reason': 'recording_disabled'})
            continue
        bag = artifact_path(campaign, entry['bag_path'])
        _validate_selected_entry(campaign, bag, entry, seen)
        selected.append(entry)
    excluded.extend(_unselected_bags(campaign, seen))
    return selected, excluded


def _validate_selected_entry(campaign, bag, entry, seen):
    if bag.parent != (campaign / 'rosbags').resolve():
        raise ValueError(f'Bag is not in accepted root: {bag}')
    if not (bag / 'metadata.yaml').is_file():
        raise ValueError(f'Missing accepted recording metadata: {bag}')
    if bag in seen:
        raise ValueError(f'Duplicate bag ownership: {bag}')
    if not entry.get('attempt_id') or not entry.get('adaptation_manager'):
        raise ValueError(f'Incomplete selection entry: {entry}')
    seen.add(bag)


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
