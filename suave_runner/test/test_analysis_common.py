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

"""Test the shared pipeline-found parser and accepted-attempt selection."""

import numpy as np

import pandas as pd

import pytest

from suave_runner.analysis.common import parse_pipeline_found
from suave_tools.runs import accepted_attempts


def test_pipeline_found_accepts_boolean_spellings():
    """Map true/false/1/0/yes/no in any case and spacing."""
    values = pd.Series([' True', 'false', '1', '0', 'YES', 'no', True, 0])
    assert parse_pipeline_found(values, 'input').tolist() == [
        True, False, True, False, True, False, True, False]


@pytest.mark.parametrize('value', ['unknown', '', np.nan, None, '2'])
def test_pipeline_found_rejects_other_values(value):
    """Never guess a detection status."""
    with pytest.raises(ValueError, match='invalid pipeline found'):
        parse_pipeline_found(pd.Series(['true', value]), 'input')


def test_accepted_attempts_maps_completed_runs(tmp_path, attempt):
    """Return each manager's completed attempt, skipping failed runs."""
    attempt(tmp_path, 'run_0_0', enabled=False, outcome='failed')
    _, first = attempt(tmp_path, 'run_0_0', enabled=False,
                       outcome='completed')
    attempt(tmp_path, 'run_0_1', enabled=False, outcome='failed')
    _, other = attempt(tmp_path, 'run_1_1', enabled=False,
                       outcome='completed', manager='rebetmc')
    assert accepted_attempts(tmp_path) == {
        'bt': {0: first['attempt_id']},
        'rebetmc': {1: other['attempt_id']},
    }


def test_accepted_attempts_rejects_manager_in_two_experiments(
        tmp_path, attempt):
    """A manager label must identify one experiment of the campaign."""
    attempt(tmp_path, 'run_0_0', enabled=False, outcome='completed')
    attempt(tmp_path, 'run_1_0', enabled=False, outcome='completed')
    with pytest.raises(ValueError, match='appears in experiments'):
        accepted_attempts(tmp_path)
