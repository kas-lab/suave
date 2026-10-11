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

"""Shared fixtures for runs.json attempts and their metrics CSVs."""

import pandas as pd
import pytest

from suave_runner.run_artifacts import attempt_details
from suave_runner.run_artifacts import finalize_attempt
from suave_runner.run_status import open_runs
from suave_runner.run_status import start_attempt
from suave_tools.runs import bag_dir
from suave_tools.runs import write_runs


def valid_bag(path):
    """Accept any bag folder that has metadata."""
    assert (path / 'metadata.yaml').is_file()


@pytest.fixture
def attempt():
    """Start (and optionally finish) an attempt in a campaign's runs.json."""
    def start(campaign, run_id='run_0_0', enabled=True, bag=True,
              outcome=None, failure_type='', validate=valid_bag,
              manager='bt', mission='suave'):
        runs = open_runs(campaign)
        details = attempt_details(run_id, manager, mission, enabled)
        entry = start_attempt(runs, run_id, manager, mission,
                              'ros2 launch pkg file', **details)
        write_runs(campaign, runs)
        if enabled and bag:
            pending = bag_dir(campaign, run_id, entry, 'pending')
            pending.mkdir(parents=True)
            (pending / 'metadata.yaml').write_text('fixture')
        if outcome is not None:
            finalize_attempt(campaign, runs, run_id, outcome, validate,
                             failure_type, failure_type)
        return runs, entry

    return start


@pytest.fixture
def metrics_campaign(attempt):
    """Write a campaign's runs.json and one raw metrics CSV per manager."""
    def build(campaign, frames, retried=(), mission='suave'):
        # frames maps manager -> rows with run_idx, in experiment order. A
        # retried (manager, run_idx) first gets a failed attempt whose row
        # holds sentinel 999 metrics, as a broken run would leave behind.
        campaign.mkdir(parents=True, exist_ok=True)
        for exp_idx, (manager, frame) in enumerate(frames.items()):
            rows = []
            for row in frame.to_dict('records'):
                run_id = f'run_{exp_idx}_{row["run_idx"]}'
                if (manager, row['run_idx']) in retried:
                    _, failed = attempt(
                        campaign, run_id, enabled=False, outcome='failed',
                        failure_type='broken_run', manager=manager,
                        mission=mission)
                    rows.append({
                        **{key: 999 for key in row},
                        'run_idx': row['run_idx'],
                        'pipeline found': True,
                        'attempt_id': failed['attempt_id']})
                _, entry = attempt(
                    campaign, run_id, enabled=False, outcome='completed',
                    manager=manager, mission=mission)
                rows.append({**row, 'attempt_id': entry['attempt_id']})
            pd.DataFrame(rows).to_csv(
                campaign / f'{manager}_{mission}.csv', index=False)
        return campaign

    return build
