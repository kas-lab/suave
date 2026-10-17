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

"""Test alternative search-footprint export helpers."""

import importlib.util
import sys
from pathlib import Path  # noqa: I100

import numpy as np


def load_exporter(monkeypatch):
    """Load the search-footprint export script as a module."""
    script_dir = Path(__file__).parents[1] / 'scripts'
    monkeypatch.syspath_prepend(str(script_dir))
    script = script_dir / 'export_search_footprint_alternatives.py'
    spec = importlib.util.spec_from_file_location('search_exporter', script)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_accumulated_values_integrate_until_detection(monkeypatch):
    """Verify accumulation uses active search time only."""
    exporter = load_exporter(monkeypatch)
    run = np.asarray([0.0, 0.5, 1.0, np.nan])
    grid = np.asarray([0.0, 25.0, 50.0, 75.0])
    auc, active_average = exporter.accumulated_values(
        run, grid, detection_percent=50.0, detection_time_s=100.0)
    assert auc == 50.0
    assert active_average == 0.5


def test_finite_table_rows_skip_censored_values(monkeypatch):
    """Verify censored values do not appear in exported rows."""
    exporter = load_exporter(monkeypatch)
    rows = exporter.finite_table_rows(
        np.asarray([0.0, 50.0, 100.0]),
        np.asarray([0.2, np.nan, 0.8]),
        np.asarray([0.1, 0.0, 0.3]),
        np.asarray([2, 0, 3]))
    assert rows == [{
        'time_percent': 0.0,
        'mean': 0.2,
        'ci_low': 0.1,
        'ci_high': 0.30000000000000004,
        'ci': 0.1,
        'count': 2,
    }, {
        'time_percent': 100.0,
        'mean': 0.8,
        'ci_low': 0.5,
        'ci_high': 1.0,
        'ci': 0.3,
        'count': 3,
    }]


def test_load_detection_times_uses_found_runs(monkeypatch, tmp_path):
    """Verify detection times come from successful pipeline detections."""
    exporter = load_exporter(monkeypatch)
    path = tmp_path / 'bt_suave.csv'
    path.write_text(
        'pipeline found,time searching pipeline (s),mission duration (s),'
        'run_idx,attempt_id\n'
        'True,40,250,0,a1\n'
        'False,250,250,1,a2\n', encoding='utf-8')
    detections = exporter.load_detection_times(tmp_path, ['bt'])
    assert detections == {
        'bt': {0: {
            'attempt_id': 'a1',
            'search_time_s': 40.0,
            'mission_duration_s': 250.0,
        }}
    }
