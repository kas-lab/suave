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

"""Own attempt bags and finish or recover attempts recorded in runs.json."""

from contextlib import contextmanager
import fcntl
from pathlib import Path
import re
import shlex
import uuid

from suave_runner.run_status import finish_attempt
from suave_tools.runs import bag_dir
from suave_tools.runs import log_dir
from suave_tools.runs import RUN_ID
from suave_tools.runs import write_runs

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
    """Allocate a new attempt identity before starting any processes."""
    if not RUN_ID.fullmatch(run_id) or any(
            not COMPONENT.fullmatch(value) for value in (manager, mission)):
        raise ValueError('Unsafe trial, manager, or mission name')
    return {'attempt_id': uuid.uuid4().hex,
            'bag': 'pending' if enabled else None}


def _move(source, target):
    if source.exists() and target.exists():
        raise RuntimeError(
            f'Both artifact locations exist: {source}, {target}')
    if source.exists():
        target.parent.mkdir(parents=True, exist_ok=True)
        source.rename(target)


def finalize_attempt(campaign, runs, run_id, outcome, validate,
                     reason='', failure_type=''):
    """Move a stopped attempt's bag, then persist its outcome in one write."""
    if outcome not in ('completed', 'failed', 'interrupted'):
        raise ValueError(f'Invalid attempt outcome: {outcome}')
    entry = runs['runs'][run_id]
    bag = None
    if entry['bag'] is not None:
        outcome, bag, reason, failure_type = _finalize_recording_attempt(
            campaign, run_id, entry, validate, outcome, reason, failure_type)
    finish_attempt(entry, outcome, bag, reason, failure_type)
    write_runs(campaign, runs)


def _finalize_recording_attempt(campaign, run_id, entry, validate, outcome,
                                reason, failure_type):
    pending = bag_dir(campaign, run_id, entry, 'pending')
    if outcome == 'completed':
        accepted = bag_dir(campaign, run_id, entry, 'accepted')
        if pending.exists() and accepted.exists():
            raise RuntimeError('Both pending and accepted bag exist')
        try:
            validate(pending)
        except Exception as error:  # noqa: B902
            outcome, reason, failure_type = (
                'failed', str(error), 'recording_invalid')
        else:
            _move(pending, accepted)
            return outcome, 'accepted', reason, failure_type
    # 'failed' also covers a recorder that never wrote anything.
    _move(pending, bag_dir(campaign, run_id, entry, 'failed'))
    return outcome, 'failed', reason, failure_type


def recover_attempts(campaign, runs, validate, confirm_stopped):
    """Settle attempts a previous runner left unfinished before retrying."""
    for run_id, entry in sorted(runs['runs'].items()):
        if entry['status'] == 'completed':
            _check_completed(campaign, run_id, entry, validate)
        elif entry['status'] == 'running' or entry['bag'] == 'pending':
            _recover_record(campaign, run_id, entry, validate,
                            confirm_stopped)
            write_runs(campaign, runs)


def _check_completed(campaign, run_id, entry, validate):
    # Resume skips completed runs, so their bags must really be accepted.
    if entry['bag'] is None:
        return
    if entry['bag'] != 'accepted':
        raise RuntimeError(
            f'Completed run without accepted bag; not rerunning: {run_id}')
    bag = bag_dir(campaign, run_id, entry)
    if not bag.is_dir():
        raise RuntimeError(
            f'Checkpointed bag is missing; not rerunning: {bag}')
    validate(bag)


def _recover_record(campaign, run_id, entry, validate, confirm_stopped):
    """Decide an unfinished attempt's outcome from where its bag is."""
    confirm_stopped(log_dir(campaign, run_id, entry))
    if entry['status'] == 'running':
        reason, failure_type = (
            'Runner stopped before a durable outcome', 'interrupted')
    else:
        reason, failure_type = entry['failure_reason'], entry['failure_type']
    if entry['bag'] is None:
        finish_attempt(entry, 'interrupted', None, reason, failure_type)
        return
    pending = bag_dir(campaign, run_id, entry, 'pending')
    accepted = bag_dir(campaign, run_id, entry, 'accepted')
    failed = bag_dir(campaign, run_id, entry, 'failed')
    if pending.exists() and accepted.exists():
        raise RuntimeError(
            f'Both artifact locations exist: {pending}, {accepted}')
    if accepted.exists():
        # Only a completed, validated attempt is ever moved here.
        try:
            validate(accepted)
        except Exception as error:  # noqa: B902
            _move(accepted, failed)
            finish_attempt(entry, 'failed', 'failed', str(error),
                           'recording_invalid')
        else:
            finish_attempt(entry, 'completed', 'accepted')
        return
    _move(pending, failed)
    finish_attempt(entry, 'interrupted', 'failed', reason, failure_type)
