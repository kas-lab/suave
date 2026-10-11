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


def native_entry(campaign, record):
    """Validate a checkpointed v2 status and return its selected bag."""
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
        'source': 'status_v2',
    }


def select_accepted_bags(campaign):
    """Return checkpointed native or legacy recordings."""
    campaign = Path(campaign)
    markers = {path.stem for path in campaign.glob('run_*_*.done')}
    legacy, rejected = _load_legacy_manifest(campaign, markers)
    # Rejected legacy attempts stay reported even after their trial is
    # un-checkpointed for a rerun.
    excluded = [{key: entry[key] for key in ('run_id', 'bag_path', 'reason')
                 if key in entry} for entry in rejected.values()]
    selected, seen, unmapped = [], set(), []
    for run_id in sorted(markers):
        entry = _entry_for_run(campaign, run_id, legacy, rejected, excluded)
        if entry is None:
            if any(item.get('run_id') == run_id for item in excluded):
                continue
            unmapped.append(run_id)
            continue
        bag = artifact_path(campaign, entry['bag_path'])
        _validate_selected_entry(campaign, bag, entry, seen)
        selected.append(entry)
    if unmapped:
        raise ValueError('Unmapped legacy trials: ' + ', '.join(unmapped))
    excluded.extend(_unselected_bags(campaign, seen))
    return selected, excluded


def _load_legacy_manifest(campaign, markers):
    manifest = campaign / 'accepted_bags.json'
    if not manifest.exists():
        return {}, {}
    data = json.loads(manifest.read_text())
    if (data.get('schema_version') != 1 or
            data.get('source') != 'legacy_reconciliation'):
        raise ValueError(f'Unsupported legacy manifest: {manifest}')
    legacy, rejected = {}, {}
    for entry in data['entries']:
        run_id = entry['run_id']
        if run_id in legacy or run_id not in markers:
            raise ValueError(
                f'Duplicate/uncheckpointed legacy trial: {run_id}')
        legacy[run_id] = entry
    for entry in data.get('excluded', []):
        run_id = entry['run_id']
        if not entry.get('reason'):
            raise ValueError(f'Exclusion without reason: {run_id}')
        if run_id in legacy or run_id in rejected:
            raise ValueError(f'Duplicate legacy exclusion: {run_id}')
        rejected[run_id] = entry
    return legacy, rejected


def _entry_for_run(campaign, run_id, legacy, rejected, excluded):
    if not RUN_ID.fullmatch(run_id):
        raise ValueError(f'Invalid checkpoint: {run_id}')
    path = campaign / f'{run_id}.status.json'
    record = json.loads(path.read_text()) if path.exists() else {}
    version = record.get('schema_version', 1)
    if version not in (1, 2):
        raise ValueError(f'Unsupported status schema: {path}')
    if version == 1:
        # A rejected legacy trial is never selected, checkpointed or not.
        return None if run_id in rejected else legacy.get(run_id)
    if record['run_id'] != run_id:
        raise ValueError(f'Mismatched status: {path}')
    entry = native_entry(campaign, record)
    if run_id in legacy and (
            entry is None or legacy[run_id]['bag_path'] != entry['bag_path']):
        raise ValueError(f'Conflicting legacy ownership: {run_id}')
    if entry is None:
        excluded.append({'run_id': run_id, 'reason': 'recording_disabled'})
    return entry


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
