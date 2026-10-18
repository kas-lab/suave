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

"""Test PGFPlots data export helpers."""

import importlib.util
import sys
from pathlib import Path  # noqa: I100

import numpy as np


def load_exporter(monkeypatch):
    """Load the TikZ export script as a module."""
    script_dir = Path(__file__).parents[1] / 'scripts'
    monkeypatch.syspath_prepend(str(script_dir))
    script = script_dir / 'export_requirement_fulfillment_tikz.py'
    spec = importlib.util.spec_from_file_location('tikz_exporter', script)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_table_rows_clip_confidence_interval_and_skip_missing(monkeypatch):
    """Verify PGFPlots rows contain finite clipped intervals only."""
    exporter = load_exporter(monkeypatch)
    rows = exporter.table_rows(
        np.asarray([0.0, 50.0, 100.0]),
        np.asarray([np.nan, 0.5, 0.95]),
        np.asarray([0.0, 0.2, 0.2]),
        np.asarray([0, 4, 5]))
    assert rows == [{
        'time_percent': 50.0,
        'mean': 0.5,
        'ci_low': 0.3,
        'ci_high': 0.7,
        'ci': 0.2,
        'count': 4,
    }, {
        'time_percent': 100.0,
        'mean': 0.95,
        'ci_low': 0.75,
        'ci_high': 1.0,
        'ci': 0.2,
        'count': 5,
    }]


def test_write_table_uses_pgfplots_header(monkeypatch, tmp_path):
    """Verify exported tables are whitespace-delimited with a header."""
    exporter = load_exporter(monkeypatch)
    path = tmp_path / 'data' / 'requirement_manager.dat'
    exporter.write_table(path, [{
        'time_percent': 12.34567,
        'mean': 0.5,
        'ci_low': 0.25,
        'ci_high': 0.75,
        'ci': 0.25,
        'count': 3,
    }], precision=3)
    assert path.read_text(encoding='utf-8').splitlines() == [
        'time_percent mean ci_low ci_high ci count',
        '12.346 0.500 0.250 0.750 0.250 3',
    ]


def test_export_requirement_tables_writes_one_file_per_manager(monkeypatch,
                                                               tmp_path):
    """Verify requirement export skips managers without observations."""
    exporter = load_exporter(monkeypatch)
    grid = np.asarray([0.0, 50.0, 100.0])
    runs = {
        'rebetmc': [np.asarray([np.nan, 0.5, 1.0])],
        'random': [],
    }
    entries = exporter.export_requirement_tables(
        'thruster_availability', runs, ['rebetmc', 'random'], grid, tmp_path,
        precision=2)
    assert sorted(entries) == ['rebetmc']
    assert entries['rebetmc']['file'] == (
        'data/thruster_availability_rebetmc.dat')
    table = tmp_path / entries['rebetmc']['file']
    assert table.is_file()
    assert len(table.read_text(encoding='utf-8').splitlines()) == 3
