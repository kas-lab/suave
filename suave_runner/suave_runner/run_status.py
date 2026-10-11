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

"""Atomic status sidecars preserving attempt history."""

from datetime import datetime
import json
from pathlib import Path
import time

from suave_tools.bag_selection import load_status
from suave_tools.bag_selection import SCHEMA_VERSION


def timestamp():
    """Return an unambiguous wall-clock timestamp."""
    return datetime.now().astimezone().isoformat()


def write_status(path, record):
    """Atomically replace a status sidecar, never a completion marker."""
    path = Path(path)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(record, indent=2) + '\n')
    temporary.replace(path)


def start_status(result_path, exp_idx, run_idx, manager, mission, **details):
    """Persist a new running attempt while retaining previous attempts."""
    if 'attempt_id' not in details:
        raise ValueError('A status needs attempt_details() ownership')
    run_id = f'run_{exp_idx}_{run_idx}'
    path = Path(result_path) / f'{run_id}.status.json'
    attempts = []
    if path.exists():
        previous = json.loads(path.read_text())
        attempts = previous.pop('attempts', [])
        attempts.append(previous)
    record = dict(details, schema_version=SCHEMA_VERSION, run_id=run_id,
                  experiment_index=exp_idx, run_index=run_idx,
                  adaptation_manager=manager, mission_name=mission,
                  result_path=str(result_path),
                  started_at=timestamp(), started_epoch=time.time(),
                  finished_at=None, finished_epoch=None, elapsed_seconds=None,
                  status='running', success=False, failure_type='',
                  failure_reason='', attempts=attempts)
    write_status(path, record)
    return path, record


def finish_status(path, record, status, reason='', failure_type=''):
    """Persist the terminal outcome of the current attempt."""
    record.update(status=status, success=status == 'completed',
                  failure_reason=reason, failure_type=failure_type,
                  finished_at=timestamp(), finished_epoch=time.time())
    record['elapsed_seconds'] = round(
        record['finished_epoch'] - record['started_epoch'], 3)
    write_status(path, record)


def load_statuses(result_path):
    """Read current status records, ignoring invalid sidecars."""
    records = []
    for path in Path(result_path).glob('run_*_*.status.json'):
        try:
            records.append(load_status(path))
        except (OSError, ValueError):
            continue
    return records
