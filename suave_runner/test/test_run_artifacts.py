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


"""Test attempt transactions, ownership, and checkpoint selection."""

import json

import pytest

from suave_runner.run_artifacts import artifact_path
from suave_runner.run_artifacts import attempt_details
from suave_runner.run_artifacts import campaign_lock
from suave_runner.run_artifacts import finalize_attempt
from suave_runner.run_artifacts import recording_enabled
from suave_runner.run_artifacts import recover_attempts
from suave_runner.run_status import start_status
from suave_runner.run_status import write_status
from suave_tools.bag_selection import select_accepted_bags


def attempt(tmp_path, enabled=True):
    details = attempt_details('run_0_0', 'bt', 'suave', enabled)
    path, record = start_status(tmp_path, 0, 0, 'bt', 'suave', **details)
    if enabled:
        bag = artifact_path(tmp_path, record['bag']['pending_path'])
        bag.mkdir(parents=True)
        (bag / 'metadata.yaml').write_text('fixture')
    record['execution_status'] = 'completed'
    write_status(path, record)
    return path, record


def valid(path):
    assert (path / 'metadata.yaml').is_file()


def stopped(path):
    pass


def test_accept_and_select(tmp_path):
    path, record = attempt(tmp_path)
    finalize_attempt(tmp_path, path, record, valid)
    entries, exclusions = select_accepted_bags(tmp_path)
    assert len(entries) == 1 and not exclusions
    assert entries[0]['attempt_id'] == record['attempt_id']
    assert (tmp_path / 'run_0_0.done').exists()
    recover_attempts(tmp_path, valid, stopped)


def test_retry_has_new_identity_and_history(tmp_path):
    path, record = attempt(tmp_path)
    record['execution_status'] = 'failed'
    finalize_attempt(tmp_path, path, record, valid)
    _, retry = attempt(tmp_path)
    assert retry['attempt_id'] != record['attempt_id']
    assert retry['attempts'][0]['bag']['state'] == 'failed'
    assert not (tmp_path / 'run_0_0.done').exists()


def test_invalid_success_quarantined(tmp_path):
    path, record = attempt(tmp_path)

    def invalid(path):
        raise ValueError('bad bag')

    finalize_attempt(tmp_path, path, record, invalid)
    assert record['execution_status'] == 'completed'
    assert record['status'] == 'failed'
    assert record['failure_type'] == 'recording_invalid'
    assert record['bag']['state'] == 'failed'
    assert not (tmp_path / 'run_0_0.done').exists()


@pytest.mark.parametrize('moved', [False, True])
def test_recover_promotion(tmp_path, moved):
    path, record = attempt(tmp_path)
    record['finalization_state'] = 'promoting'
    if moved:
        artifact_path(tmp_path, record['bag']['pending_path']).rename(
            artifact_path(tmp_path, record['bag']['accepted_path']))
    write_status(path, record)
    recover_attempts(tmp_path, valid, stopped)
    assert len(select_accepted_bags(tmp_path)[0]) == 1


def test_recover_after_status_before_checkpoint(tmp_path):
    path, record = attempt(tmp_path)
    finalize_attempt(tmp_path, path, record, valid)
    (tmp_path / 'run_0_0.done').unlink()
    recover_attempts(tmp_path, valid, stopped)
    assert (tmp_path / 'run_0_0.done').exists()


def test_running_attempt_recovers_as_interrupted(tmp_path):
    path, record = attempt(tmp_path)
    record['execution_status'] = 'running'
    write_status(path, record)
    recover_attempts(tmp_path, valid, stopped)
    recovered = json.loads(path.read_text())
    assert recovered['status'] == 'interrupted'
    assert recovered['bag']['state'] == 'failed'


def test_live_writer_blocks_recovery(tmp_path):
    path, record = attempt(tmp_path)

    def live(path):
        raise RuntimeError('writer alive')

    with pytest.raises(RuntimeError, match='writer alive'):
        recover_attempts(tmp_path, valid, live)
    assert artifact_path(tmp_path, record['bag']['pending_path']).exists()
    assert not (tmp_path / 'run_0_0.done').exists()


def test_conflicting_paths_stop(tmp_path):
    path, record = attempt(tmp_path)
    artifact_path(tmp_path, record['bag']['accepted_path']).mkdir()
    with pytest.raises(RuntimeError, match='Both'):
        finalize_attempt(tmp_path, path, record, valid)


@pytest.mark.parametrize('key', ['pending_path', 'accepted_path'])
def test_foreign_bag_path_is_not_promoted(tmp_path, key):
    path, record = attempt(tmp_path)
    record['finalization_state'] = 'promoting'
    record['bag'][key] = 'rosbags/bt_suave_run_0_0_' + '0' * 32
    write_status(path, record)
    with pytest.raises(RuntimeError, match='do not belong'):
        recover_attempts(tmp_path, valid, stopped)
    assert not (tmp_path / 'run_0_0.done').exists()


def test_unconfirmed_shutdown_quarantined_on_recovery(tmp_path):
    path, record = attempt(tmp_path)
    record.update(execution_status='interrupted',
                  failure_type='shutdown_unconfirmed')
    write_status(path, record)
    recover_attempts(tmp_path, valid, stopped)
    recovered = json.loads(path.read_text())
    assert recovered['status'] == 'interrupted'
    assert recovered['failure_type'] == 'shutdown_unconfirmed'
    assert recovered['bag']['state'] == 'failed'
    assert artifact_path(tmp_path, recovered['bag']['failed_path']).exists()
    assert not (tmp_path / 'run_0_0.done').exists()


def test_checkpoint_with_missing_bag_is_integrity_error(tmp_path):
    path, record = attempt(tmp_path)
    finalize_attempt(tmp_path, path, record, valid)
    artifact_path(tmp_path, record['bag']['accepted_path']).rename(
        tmp_path / 'elsewhere')
    with pytest.raises(RuntimeError, match='Checkpointed bag is missing'):
        recover_attempts(tmp_path, valid, stopped)
    assert (tmp_path / 'run_0_0.done').exists()


def test_disabled_recording_checkpoint(tmp_path):
    path, record = attempt(tmp_path, False)
    finalize_attempt(tmp_path, path, record, valid)
    entries, exclusions = select_accepted_bags(tmp_path)
    assert not entries and exclusions[0]['reason'] == 'recording_disabled'


def test_checkpoint_conflicting_with_status_is_rejected(tmp_path):
    path, record = attempt(tmp_path)
    (tmp_path / 'run_0_0.done').touch()
    with pytest.raises(ValueError, match='Conflicting'):
        select_accepted_bags(tmp_path)


def test_unsupported_status_schema_is_rejected(tmp_path):
    path, record = attempt(tmp_path)
    finalize_attempt(tmp_path, path, record, valid)
    record['schema_version'] = 2
    write_status(path, record)
    with pytest.raises(ValueError, match='Unsupported status schema'):
        recover_attempts(tmp_path, valid, stopped)
    with pytest.raises(ValueError, match='Unsupported status schema'):
        select_accepted_bags(tmp_path)


def test_incomplete_status_is_rejected(tmp_path):
    write_status(tmp_path / 'run_0_0.status.json', {
        'schema_version': 1, 'run_id': 'run_0_0',
        'status': 'completed', 'success': True})
    (tmp_path / 'run_0_0.done').touch()
    with pytest.raises(ValueError, match='Incomplete status'):
        recover_attempts(tmp_path, valid, stopped)
    with pytest.raises(ValueError, match='Incomplete status'):
        select_accepted_bags(tmp_path)


def test_checkpoint_without_status_is_rejected(tmp_path):
    (tmp_path / 'run_0_0.done').touch()
    with pytest.raises(ValueError, match='without status'):
        select_accepted_bags(tmp_path)


def test_status_requires_attempt_ownership(tmp_path):
    with pytest.raises(ValueError, match='attempt_details'):
        start_status(tmp_path, 0, 0, 'bt', 'suave')


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
