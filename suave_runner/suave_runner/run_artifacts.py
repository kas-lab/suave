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

"""Persist attempt ownership and select only checkpointed recordings."""

from contextlib import contextmanager
import fcntl
import json
from pathlib import Path
import re
import shlex
import uuid

from suave_runner.run_status import finish_status
from suave_runner.run_status import write_status
from suave_tools.bag_selection import artifact_path
from suave_tools.bag_selection import native_entry

RUN_ID = re.compile(r'run_\d+_\d+')
COMPONENT = re.compile(r'[A-Za-z0-9_-]+')


@contextmanager
def campaign_lock(campaign):
    """Exclude concurrent runners without treating stale files as locks."""
    with (Path(campaign) / '.runner.lock').open('a') as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError(
                f'Campaign already in use: {campaign}') from error
        try:
            yield
        finally:
            fcntl.flock(handle, fcntl.LOCK_UN)


def recording_enabled(command, default=True):
    """Resolve the experiment recording override and reject owned paths."""
    arguments = {
        part.split(':=', 1)[0]: part.split(':=', 1)[1]
        for part in shlex.split(command)[4:] if ':=' in part
    }
    if 'bag_output_path' in arguments:
        raise ValueError('The runner owns bag_output_path')
    value = arguments.get('record_bags', str(default)).lower()
    if value not in ('true', 'false', '1', '0'):
        raise ValueError(f'Invalid record_bags: {value}')
    return value in ('true', '1')


def attempt_details(run_id, manager, mission, enabled):
    """Allocate immutable attempt paths before starting any processes."""
    if not RUN_ID.fullmatch(run_id) or any(
            not COMPONENT.fullmatch(value) for value in (manager, mission)):
        raise ValueError('Unsafe trial, manager, or mission name')
    attempt_id = uuid.uuid4().hex
    name = f'{manager}_{mission}_{run_id}_{attempt_id}'
    pending = f'rosbags/pending/{name}' if enabled else None
    return {
        'schema_version': 2,
        'attempt_id': attempt_id,
        'recording_enabled': enabled,
        'execution_status': 'running',
        'finalization_state': 'pending',
        'log_dir': f'logs/{run_id}/{attempt_id}',
        'bag': {
            'pending_path': pending,
            'accepted_path': f'rosbags/{name}' if enabled else None,
            'failed_path': f'rosbags/failed/{name}' if enabled else None,
            'path': pending,
            'state': 'pending' if enabled else 'disabled',
            'validation_error': '',
        },
    }


def _checkpoint(campaign, record):
    marker = Path(campaign) / f'{record["run_id"]}.done'
    temporary = marker.with_suffix('.done.tmp')
    temporary.write_text('')
    temporary.replace(marker)


def _move(campaign, bag, destination):
    source = artifact_path(campaign, bag['pending_path'])
    target = artifact_path(campaign, bag[destination])
    if source.exists() and target.exists():
        raise RuntimeError(
            f'Both artifact locations exist: {source}, {target}')
    if source.exists():
        target.parent.mkdir(parents=True, exist_ok=True)
        source.rename(target)
    bag['path'] = bag[destination] if target.exists() else bag['pending_path']
    return target.exists()


def finalize_attempt(campaign, path, record, validate):
    """Finalize a stopped attempt idempotently, committing its marker last."""
    if record['execution_status'] == 'running':
        raise ValueError('Persist an execution outcome before finalization')
    accepted = record['execution_status'] == 'completed'
    if record['recording_enabled']:
        accepted = _finalize_recording_attempt(
            campaign, path, record, validate, accepted)
    record['finalization_state'] = 'finished'
    status = 'completed' if accepted else _failure_status(record)
    finish_status(path, record, status, record.get('failure_reason', ''),
                  record.get('failure_type', ''))
    if accepted:
        _checkpoint(campaign, record)


def _failure_status(record):
    if record['execution_status'] == 'interrupted':
        return 'interrupted'
    return 'failed'


def _check_ownership(record):
    """Require every persisted bag path to name this exact attempt."""
    bag = record['bag']
    name = Path(bag['pending_path']).name
    expected = {
        'pending_path': f'rosbags/pending/{name}',
        'accepted_path': f'rosbags/{name}',
        'failed_path': f'rosbags/failed/{name}',
    }
    owned = name.endswith(f'_{record["run_id"]}_{record["attempt_id"]}')
    if not owned or any(bag[key] != value for key, value in expected.items()):
        raise RuntimeError(
            f'Bag paths do not belong to attempt {record["attempt_id"]}')


def _finalize_recording_attempt(campaign, path, record, validate, accepted):
    _check_ownership(record)
    bag = record['bag']
    if accepted and record['finalization_state'] != 'quarantining':
        source = artifact_path(campaign, bag['pending_path'])
        target = artifact_path(campaign, bag['accepted_path'])
        if source.exists() and target.exists():
            raise RuntimeError('Both pending and accepted bag exist')
        try:
            validate(source if source.exists() else target)
        except Exception as error:  # noqa: B902
            accepted = False
            bag['validation_error'] = str(error)
            record.update(failure_type='recording_invalid',
                          failure_reason=str(error))
            if target.exists():
                raise RuntimeError(
                    f'Promoted bag failed validation: {target}') from error
        else:
            bag['state'] = 'validated'
            record['finalization_state'] = 'promoting'
            write_status(path, record)
            if not _move(campaign, bag, 'accepted_path'):
                raise RuntimeError('Validated bag disappeared')
            bag['state'] = 'accepted'
    else:
        accepted = False
    if not accepted:
        record['finalization_state'] = 'quarantining'
        write_status(path, record)
        exists = _move(campaign, bag, 'failed_path')
        bag['state'] = 'failed' if exists else 'missing'
    return accepted


def recover_attempts(campaign, validate, confirm_stopped):
    """Finish interrupted transactions before retries."""
    for path in sorted(Path(campaign).glob('run_*_*.status.json')):
        record = json.loads(path.read_text())
        if record.get('schema_version') == 1:
            continue
        if record.get('schema_version') != 2:
            raise ValueError(f'Unsupported status schema: {path}')
        if record['run_id'] != path.name.removesuffix('.status.json'):
            raise ValueError(f'Mismatched trial identity: {path}')
        _recover_record(campaign, path, record, validate, confirm_stopped)


def _recover_record(campaign, path, record, validate, confirm_stopped):
    marker = Path(campaign) / f'{record["run_id"]}.done'
    if marker.exists():
        native_entry(campaign, record)
        if record['recording_enabled']:
            _check_ownership(record)
            bag = artifact_path(campaign, record['bag']['path'])
            if not bag.is_dir():
                raise RuntimeError(
                    f'Checkpointed bag is missing; not rerunning: {bag}')
            validate(bag)
        return
    if record['finalization_state'] == 'finished' and not record['success']:
        return
    confirm_stopped(artifact_path(campaign, record['log_dir']))
    if record['execution_status'] == 'running':
        record.update(
            execution_status='interrupted',
            failure_type='interrupted',
            failure_reason='Runner stopped before a durable outcome')
        write_status(path, record)
    finalize_attempt(campaign, path, record, validate)
