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

"""Shared fixtures for runs.json attempts."""

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
              outcome=None, failure_type='', validate=valid_bag):
        runs = open_runs(campaign)
        details = attempt_details(run_id, 'bt', 'suave', enabled)
        entry = start_attempt(runs, run_id, 'bt', 'suave',
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
