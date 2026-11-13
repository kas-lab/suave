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

"""Test representative-run export helpers."""

import importlib.util
import sys
from pathlib import Path  # noqa: I100

import pytest


def load_exporter(monkeypatch):
    """Load the representative-run export script as a module."""
    script_dir = Path(__file__).parents[1] / 'scripts'
    monkeypatch.syspath_prepend(str(script_dir))
    script = script_dir / 'export_representative_run_tikz.py'
    spec = importlib.util.spec_from_file_location('run_exporter', script)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def candidate(run_id, search, distance, reaction):
    """Return one selection candidate."""
    return {'run_id': run_id, 'search_time_s': search,
            'distance_inspected_m': distance,
            'mean_reaction_time_s': reaction}


def test_choose_run_picks_run_closest_to_median(monkeypatch):
    """Verify the default choice minimizes the summed median distance."""
    exporter = load_exporter(monkeypatch)
    candidates = [
        candidate('run_0_0', 40.0, 60.0, 1.5),
        candidate('run_0_1', 100.0, 45.0, 1.0),
        candidate('run_0_2', 110.0, 40.0, 0.9),
    ]
    chosen, ranked = exporter.choose_run(candidates)
    assert chosen['run_id'] == 'run_0_1'
    assert [c['run_id'] for c in ranked][0] == 'run_0_1'
    assert exporter.choose_run(candidates, 'run_0_2')[0]['run_id'] == (
        'run_0_2')
    with pytest.raises(ValueError):
        exporter.choose_run(candidates, 'run_0_9')
    with pytest.raises(ValueError):
        exporter.choose_run([])


def test_adaptations_skip_repeated_altitude_and_number_in_time(monkeypatch):
    """Verify adaptations merge altitude and lifecycle events in order."""
    exporter = load_exporter(monkeypatch)
    events = {
        'altitude': [(3.0, 3.0), (5.0, 3.0), (12.0, 2.0)],
        'maintain_motion': [(8.0, 'active'), (20.0, 'inactive')],
    }
    result = exporter.adaptations(events)
    assert [(a['index'], a['time'], a['kind'], a['detail'])
            for a in result] == [
        (1, 3.0, 'altitude', '3'),
        (2, 8.0, 'maintain_motion', 'active'),
        (3, 12.0, 'altitude', '2'),
        (4, 20.0, 'maintain_motion', 'inactive'),
    ]


def test_context_intervals_cover_phases_and_failures(monkeypatch):
    """Verify phase and thruster-failure intervals, open failures included."""
    exporter = load_exporter(monkeypatch)
    events = {
        'search_start': 3.0,
        'pipeline_detected': 50.0,
        'operational_thrusters': [(1.0, 6), (10.0, 5), (12.0, 4),
                                  (20.0, 6), (90.0, 5)],
    }
    assert exporter.context_intervals(events, 100.0, 6) == [
        {'start': 3.0, 'end': 50.0, 'kind': 'search'},
        {'start': 50.0, 'end': 100.0, 'kind': 'inspect'},
        {'start': 10.0, 'end': 20.0, 'kind': 'thruster_failure'},
        {'start': 90.0, 'end': 100.0, 'kind': 'thruster_failure'},
    ]


def test_context_intervals_without_detection_search_until_end(monkeypatch):
    """Verify a run that never detects the pipeline searches until the end."""
    exporter = load_exporter(monkeypatch)
    events = {'search_start': 3.0, 'pipeline_detected': None,
              'operational_thrusters': []}
    assert exporter.context_intervals(events, 80.0, 6) == [
        {'start': 3.0, 'end': 80.0, 'kind': 'search'}]


def test_censor_rows_holds_last_value_until_cutoff(monkeypatch):
    """Verify search-only series stop at detection with a held value."""
    exporter = load_exporter(monkeypatch)
    rows = [{'time': 1.0, 'value': 0.2}, {'time': 2.0, 'value': 0.5},
            {'time': 3.0, 'value': 1.0}]
    assert exporter.censor_rows(rows, 2.5) == [
        {'time': 1.0, 'value': 0.2}, {'time': 2.0, 'value': 0.5},
        {'time': 2.5, 'value': 0.5}]
    assert exporter.censor_rows(rows, None) == rows
    assert exporter.censor_rows(rows, 0.5) == []


def test_shift_events_moves_times_and_keeps_missing(monkeypatch):
    """Verify events are moved to mission time and None stays None."""
    exporter = load_exporter(monkeypatch)
    events = {'search_start': 4.0, 'pipeline_detected': None,
              'altitude': [(6.0, 2.0)]}
    assert exporter.shift_events(events, 4.0) == {
        'search_start': 0.0, 'pipeline_detected': None,
        'altitude': [(2.0, 2.0)]}


def test_start_rows_at_zero_holds_value_before_mission(monkeypatch):
    """Verify pre-mission rows collapse into one held row at zero."""
    exporter = load_exporter(monkeypatch)
    rows = [{'time': -2.0, 'value': 0.1}, {'time': -1.0, 'value': 0.3},
            {'time': 0.5, 'value': 0.6}]
    assert exporter.start_rows_at_zero(rows) == [
        {'time': 0.0, 'value': 0.3}, {'time': 0.5, 'value': 0.6}]
    assert exporter.start_rows_at_zero(rows[2:]) == rows[2:]


def test_caption_lists_adaptations_up_to_xmax(monkeypatch):
    """Verify the caption numbers adaptations and drops those past xmax."""
    exporter = load_exporter(monkeypatch)
    adaptation_list = [
        {'index': 1, 'time': 1.04, 'label': 'Search altitude set to 3 m'},
        {'index': 2, 'time': 112.0, 'label': 'Activated thruster recovery'},
        {'index': 3, 'time': 160.0, 'label': 'Completed thruster recovery'},
    ]
    assert exporter.caption_tex(adaptation_list, 150.0) == (
        'Adaptations: (1) search altitude set to 3 m at 1.0\\,s, '
        'and (2) activated thruster recovery at 112.0\\,s.\n')


def test_preview_without_water_visibility_has_one_panel(monkeypatch):
    """Verify the water-visibility panel can be left out of the figure."""
    exporter = load_exporter(monkeypatch)
    args = ({'thruster_availability': 'data/t.dat',
             'water_visibility': 'data/w.dat'},
            ['thruster_availability'], [], [], [(3.25, 3.0)], 150.0, 'T')
    with_panel = exporter.preview_tex(*args)
    without_panel = exporter.preview_tex(*args, water_visibility=False)
    assert 'group size=1 by 2' in with_panel
    assert 'data/w.dat' in with_panel
    assert 'group size=1 by 1' in without_panel
    assert 'data/w.dat' not in without_panel
    assert 'altitude 3' not in without_panel
