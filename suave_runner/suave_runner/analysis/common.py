# Copyright 2026 KAS Lab
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

"""Shared, ROS-free helpers for reading campaign result CSVs."""

from pathlib import Path

import pandas as pd

from suave_tools.runs import accepted_attempts
from suave_tools.runs import load_runs
from suave_tools.runs import parse_run_id
from suave_tools.runs import RUNS_FILE

TRUE_VALUES = {'true', '1', 'yes'}
FALSE_VALUES = {'false', '0', 'no'}
IDENTITY_COLUMNS = ('run_idx', 'attempt_id')


def parse_pipeline_found(values, source):
    """Parse true/false/1/0/yes/no values, rejecting anything else."""
    normalized = values.map(
        lambda value: str(value).strip().lower() if not pd.isna(value) else '')
    if not normalized.isin(TRUE_VALUES | FALSE_VALUES).all():
        raise ValueError(f'{source} contains invalid pipeline found values')
    return normalized.isin(TRUE_VALUES)


def campaign_methods(campaign):
    """Return (manager, raw CSV path) per experiment, in experiment order."""
    campaign = Path(campaign)
    runs = _require_runs(campaign)
    methods = {}
    for run_id in sorted(runs, key=parse_run_id):
        entry = runs[run_id]
        methods.setdefault(entry['adaptation_manager'], campaign / (
            f'{entry["adaptation_manager"]}_{entry["mission_name"]}.csv'))
    return list(methods.items())


def accepted_rows(csv_path, manager=None):
    """Read a raw campaign CSV, keeping each run's accepted attempt only."""
    path = Path(csv_path).expanduser()
    campaign = path.parent
    _require_runs(campaign)
    if manager is None:
        managers = [name for name, method_path in campaign_methods(campaign)
                    if method_path.name == path.name]
        if not managers:
            raise ValueError(f'{path}: no experiment in {RUNS_FILE} writes it')
        manager = managers[0]
    accepted = accepted_attempts(campaign)
    if manager not in accepted:
        raise ValueError(
            f'{path}: no completed {manager!r} runs in {RUNS_FILE}; '
            f'completed managers: {sorted(accepted)}')
    accepted = accepted[manager]
    try:
        frame = pd.read_csv(path, dtype={'attempt_id': str})
    except (OSError, UnicodeError, pd.errors.ParserError,
            pd.errors.EmptyDataError) as error:
        raise ValueError(f'Cannot read CSV {path}: {error}') from error
    missing = [name for name in IDENTITY_COLUMNS if name not in frame.columns]
    if missing:
        raise ValueError(
            f'{path}: missing {missing}; only campaigns whose CSVs record '
            'run_idx and attempt_id are supported')
    run_idx = pd.to_numeric(frame['run_idx'], errors='coerce')
    keep = [
        not pd.isna(index) and index % 1 == 0 and
        accepted.get(int(index)) == attempt
        for index, attempt in zip(run_idx, frame['attempt_id'])]
    frame = frame.loc[keep].copy()
    frame['run_idx'] = run_idx[keep].astype(int)
    if frame['run_idx'].duplicated().any():
        duplicates = sorted(
            frame.loc[frame['run_idx'].duplicated(), 'run_idx'])
        raise ValueError(f'{path}: duplicate accepted rows for {duplicates}')
    absent = sorted(set(accepted) - set(frame['run_idx']))
    if absent:
        raise ValueError(f'{path}: completed runs without a row: {absent}')
    return frame.sort_values('run_idx').reset_index(drop=True)


def _require_runs(campaign):
    if not (Path(campaign) / RUNS_FILE).is_file():
        raise ValueError(f'{campaign}: no {RUNS_FILE}; not a runner campaign')
    return load_runs(campaign)['runs']
