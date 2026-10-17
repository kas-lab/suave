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

"""Run entries of a campaign's runs.json, preserving attempt history."""

from datetime import datetime
from pathlib import Path

from suave_tools.runs import load_runs
from suave_tools.runs import RUNS_FILE
from suave_tools.runs import SCHEMA_VERSION

ATTEMPT_KEYS = ('attempt_id', 'status', 'bag', 'failure_type',
                'failure_reason', 'started_at', 'finished_at')


def timestamp():
    """Return an unambiguous wall-clock timestamp."""
    return datetime.now().astimezone().isoformat(timespec='seconds')


def open_runs(campaign):
    """Load a campaign's runs.json, or start an empty one."""
    if (Path(campaign) / RUNS_FILE).exists():
        return load_runs(campaign)
    return {'schema_version': SCHEMA_VERSION, 'campaign': {}, 'runs': {}}


def start_attempt(runs, run_id, manager, mission, experiment_launch,
                  **details):
    """Make a new running attempt current, moving the previous to history."""
    if 'attempt_id' not in details:
        raise ValueError('An attempt needs attempt_details() ownership')
    attempts = []
    previous = runs['runs'].get(run_id)
    if previous is not None:
        attempts = previous['attempts']
        attempts.append({key: previous[key] for key in ATTEMPT_KEYS})
    entry = {
        'adaptation_manager': manager,
        'mission_name': mission,
        'experiment_launch': experiment_launch,
        'status': 'running',
        'attempt_id': details['attempt_id'],
        'bag': details['bag'],
        'failure_type': '',
        'failure_reason': '',
        'started_at': timestamp(),
        'finished_at': None,
        'attempts': attempts,
    }
    runs['runs'][run_id] = entry
    return entry


def finish_attempt(entry, status, bag, reason='', failure_type=''):
    """Record the terminal outcome of the current attempt."""
    entry.update(status=status, bag=bag, failure_reason=reason,
                 failure_type=failure_type, finished_at=timestamp())
