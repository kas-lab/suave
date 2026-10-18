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


"""Test attempt finalization, recovery, and accepted-run selection."""

import json

import pytest

from suave_runner.run_artifacts import attempt_details
from suave_runner.run_artifacts import campaign_lock
from suave_runner.run_artifacts import finalize_attempt
from suave_runner.run_artifacts import recording_enabled
from suave_runner.run_artifacts import recover_attempts
from suave_runner.run_status import finish_attempt
from suave_tools.runs import accepted_runs
from suave_tools.runs import artifact_path
from suave_tools.runs import bag_dir
from suave_tools.runs import load_runs
from suave_tools.runs import parse_run_id
from suave_tools.runs import write_runs


def valid(path):
    assert (path / 'metadata.yaml').is_file()


def invalid(path):
    raise ValueError('bad bag')


def stopped(path):
    pass


def recover(campaign, validate=valid, confirm=stopped):
    runs = load_runs(campaign)
    recover_attempts(campaign, runs, validate, confirm)
    return load_runs(campaign)['runs']['run_0_0']


def test_accept_and_select(tmp_path, attempt):
    _, entry = attempt(tmp_path, outcome='completed')
    stored = load_runs(tmp_path)['runs']['run_0_0']
    assert stored['status'] == 'completed' and stored['bag'] == 'accepted'
    assert bag_dir(tmp_path, 'run_0_0', entry).is_dir()
    entries, exclusions = accepted_runs(tmp_path)
    assert len(entries) == 1 and not exclusions
    assert entries[0]['attempt_id'] == entry['attempt_id']
    assert entries[0]['bag_path'] == (
        f'rosbags/bt_suave_run_0_0_{entry["attempt_id"]}')
    assert recover(tmp_path) == stored
    assert not list(tmp_path.glob('*.tmp'))


def test_retry_keeps_earlier_attempt_in_history(tmp_path, attempt):
    _, failed = attempt(tmp_path, outcome='failed', failure_type='timeout')
    runs, retry = attempt(tmp_path)
    assert retry['attempt_id'] != failed['attempt_id']
    stored = load_runs(tmp_path)['runs']['run_0_0']
    assert stored['status'] == 'running'
    [history] = stored['attempts']
    assert history['attempt_id'] == failed['attempt_id']
    assert history['status'] == 'failed' and history['bag'] == 'failed'
    assert history['failure_type'] == 'timeout'
    old_bag = bag_dir(tmp_path, 'run_0_0', stored, 'failed',
                      attempt_id=history['attempt_id'])
    assert old_bag == (tmp_path / 'rosbags' / 'failed' /
                       f'bt_suave_run_0_0_{failed["attempt_id"]}').resolve()
    assert old_bag.is_dir()
    finalize_attempt(tmp_path, runs, 'run_0_0', 'completed', valid)
    assert [e['attempt_id'] for e in accepted_runs(tmp_path)[0]] == [
        retry['attempt_id']]


def test_invalid_success_quarantined(tmp_path, attempt):
    _, entry = attempt(tmp_path, outcome='completed', validate=invalid)
    assert entry['status'] == 'failed'
    assert entry['failure_type'] == 'recording_invalid'
    assert entry['bag'] == 'failed'
    assert bag_dir(tmp_path, 'run_0_0', entry).is_dir()
    assert not bag_dir(tmp_path, 'run_0_0', entry, 'pending').exists()


def test_missing_recording_fails_completed_run(tmp_path, attempt):
    _, entry = attempt(tmp_path, bag=False, outcome='completed')
    assert entry['status'] == 'failed' and entry['bag'] == 'failed'


def crash_after_move(campaign, attempt, destination):
    """Leave runs.json 'running' with the bag already moved."""
    _, entry = attempt(campaign)
    if destination is not None:
        target = bag_dir(campaign, 'run_0_0', entry, destination)
        target.parent.mkdir(parents=True, exist_ok=True)
        bag_dir(campaign, 'run_0_0', entry, 'pending').rename(target)
    return entry


def test_recover_bag_only_accepted_and_valid(tmp_path, attempt):
    crash_after_move(tmp_path, attempt, 'accepted')
    recovered = recover(tmp_path)
    assert recovered['status'] == 'completed'
    assert recovered['bag'] == 'accepted'
    assert len(accepted_runs(tmp_path)[0]) == 1


def test_recover_bag_only_accepted_and_invalid(tmp_path, attempt):
    entry = crash_after_move(tmp_path, attempt, 'accepted')
    recovered = recover(tmp_path, invalid)
    assert recovered['status'] == 'failed'
    assert recovered['bag'] == 'failed'
    assert recovered['failure_type'] == 'recording_invalid'
    assert bag_dir(tmp_path, 'run_0_0', entry, 'failed').is_dir()
    assert not bag_dir(tmp_path, 'run_0_0', entry, 'accepted').exists()


def test_recover_bag_only_pending(tmp_path, attempt):
    entry = crash_after_move(tmp_path, attempt, None)
    recovered = recover(tmp_path)
    assert recovered['status'] == 'interrupted'
    assert recovered['failure_type'] == 'interrupted'
    assert recovered['bag'] == 'failed'
    assert bag_dir(tmp_path, 'run_0_0', entry, 'failed').is_dir()


@pytest.mark.parametrize('enabled', [True, False])
def test_recover_without_bag(tmp_path, attempt, enabled):
    attempt(tmp_path, enabled=enabled, bag=False)
    recovered = recover(tmp_path)
    assert recovered['status'] == 'interrupted'
    assert recovered['bag'] == ('failed' if enabled else None)
    assert recovered['finished_at']


def test_recover_bag_in_both_locations_raises(tmp_path, attempt):
    _, entry = attempt(tmp_path)
    bag_dir(tmp_path, 'run_0_0', entry, 'accepted').mkdir()
    with pytest.raises(RuntimeError, match='Both'):
        recover(tmp_path)
    assert load_runs(tmp_path)['runs']['run_0_0']['status'] == 'running'


def test_finalize_bag_in_both_locations_raises(tmp_path, attempt):
    runs, entry = attempt(tmp_path)
    bag_dir(tmp_path, 'run_0_0', entry, 'accepted').mkdir()
    with pytest.raises(RuntimeError, match='Both'):
        finalize_attempt(tmp_path, runs, 'run_0_0', 'completed', valid)
    assert load_runs(tmp_path)['runs']['run_0_0']['status'] == 'running'


def test_live_writer_blocks_recovery(tmp_path, attempt):
    _, entry = attempt(tmp_path)

    def live(path):
        assert path == artifact_path(
            tmp_path, f'logs/run_0_0/{entry["attempt_id"]}')
        raise RuntimeError('writer alive')

    with pytest.raises(RuntimeError, match='writer alive'):
        recover(tmp_path, confirm=live)
    assert bag_dir(tmp_path, 'run_0_0', entry, 'pending').exists()
    assert load_runs(tmp_path)['runs']['run_0_0']['status'] == 'running'


def test_unconfirmed_shutdown_quarantined_on_recovery(tmp_path, attempt):
    runs, entry = attempt(tmp_path)
    finish_attempt(entry, 'interrupted', 'pending',
                   'Launch shutdown was not confirmed', 'shutdown_unconfirmed')
    write_runs(tmp_path, runs)
    recovered = recover(tmp_path)
    assert recovered['status'] == 'interrupted'
    assert recovered['failure_type'] == 'shutdown_unconfirmed'
    assert recovered['bag'] == 'failed'
    assert bag_dir(tmp_path, 'run_0_0', recovered).is_dir()


def test_completed_run_with_missing_bag_is_integrity_error(tmp_path, attempt):
    _, entry = attempt(tmp_path, outcome='completed')
    bag_dir(tmp_path, 'run_0_0', entry).rename(tmp_path / 'elsewhere')
    with pytest.raises(RuntimeError, match='Checkpointed bag is missing'):
        recover(tmp_path)


def test_completed_run_without_accepted_bag_is_rejected(tmp_path, attempt):
    runs, entry = attempt(tmp_path)
    finish_attempt(entry, 'completed', 'pending')
    write_runs(tmp_path, runs)
    with pytest.raises(RuntimeError, match='without accepted bag'):
        recover(tmp_path)
    with pytest.raises(ValueError, match='without accepted bag'):
        accepted_runs(tmp_path)


def test_disabled_recording_is_excluded(tmp_path, attempt):
    _, entry = attempt(tmp_path, enabled=False, outcome='completed')
    assert entry['status'] == 'completed' and entry['bag'] is None
    entries, exclusions = accepted_runs(tmp_path)
    assert not entries and exclusions[0]['reason'] == 'recording_disabled'


def test_unsupported_or_incomplete_runs_are_rejected(tmp_path, attempt):
    attempt(tmp_path, outcome='completed')
    data = json.loads((tmp_path / 'runs.json').read_text())
    write_runs(tmp_path, dict(data, schema_version=1))
    with pytest.raises(ValueError, match='Unsupported runs schema'):
        accepted_runs(tmp_path)
    del data['runs']['run_0_0']['attempt_id']
    write_runs(tmp_path, data)
    with pytest.raises(ValueError, match='Incomplete run run_0_0'):
        load_runs(tmp_path)
    write_runs(tmp_path, dict(data, runs={'run_x': {}}))
    with pytest.raises(ValueError, match='Invalid run id'):
        load_runs(tmp_path)


def test_attempt_requires_safe_names_and_ownership(tmp_path):
    for args in (('run_0', 'bt', 'suave'), ('run_0_0', '../bt', 'suave'),
                 ('run_0_0', 'bt', 'a b')):
        with pytest.raises(ValueError, match='Unsafe'):
            attempt_details(*args, True)
    assert parse_run_id('run_12_3') == (12, 3)
    with pytest.raises(ValueError, match='outcome'):
        finalize_attempt(tmp_path, {'runs': {}}, 'run_0_0', 'running', valid)


def test_lock_and_path_safety(tmp_path):
    with campaign_lock(tmp_path):
        with pytest.raises(RuntimeError, match='already in use'):
            with campaign_lock(tmp_path):
                pass
    for name in ('../escape', '/tmp/escape', '.'):
        with pytest.raises(ValueError):
            artifact_path(tmp_path, name)


def test_recording_arguments():
    assert recording_enabled('ros2 launch pkg file')
    assert not recording_enabled('ros2 launch pkg file record_bags:=False')
    assert not recording_enabled('ros2 launch pkg file', False)
    with pytest.raises(ValueError):
        recording_enabled('ros2 launch pkg file record_bags:=maybe')
    with pytest.raises(ValueError):
        recording_enabled('ros2 launch pkg file bag_output_path:=/tmp/bag')
