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

"""Regression tests for read-only batch diagnosis and status summaries."""

import json

import pytest

from suave_runner.diagnose_batch import diagnose
from suave_runner.diagnose_batch import diagnose_campaign
from suave_runner.diagnose_batch import summarize
from suave_runner.run_artifacts import attempt_details
from suave_runner.run_status import finish_status
from suave_runner.run_status import start_status


def attempt(campaign, run_id, status, failure_type=''):
    exp_idx, run_idx = (int(part) for part in run_id.split('_')[1:])
    details = attempt_details(run_id, 'bt', 'suave', False)
    path, record = start_status(
        campaign, exp_idx, run_idx, 'bt', 'suave', **details)
    record.update(execution_status=status, finalization_state='finished')
    finish_status(path, record, status, failure_type, failure_type)
    if status == 'completed':
        (campaign / f'{run_id}.done').touch()
    return record


def test_retry_keeps_failed_attempt_history(tmp_path):
    attempt(tmp_path, 'run_2_0', 'failed', 'mission_done_timeout')
    attempt(tmp_path, 'run_2_0', 'completed')
    report = diagnose_campaign(tmp_path)
    assert report['historical_failed_attempts'] == 1
    assert report['completed_runs'] == 1
    assert 'suspected_causes' not in report['runs'][0]
    assert not report['warnings']


def test_failure_evidence_stops_at_teardown(tmp_path):
    record = attempt(tmp_path, 'run_0_0', 'failed', 'mission_done_timeout')
    folder = tmp_path / record['log_dir'] / 'experiment'
    folder.mkdir(parents=True)
    (folder / 'launch.log').write_text(
        '10.1 [bt] f_maintain_motion/change_mode service not available\n'
        "11.0 [launch] sending signal 'SIGINT'\n"
        '11.1 [mission_metrics] InvalidHandle\n')
    report = diagnose_campaign(tmp_path)
    assert [e['type'] for e in report['runs'][0]['suspected_causes']] == [
        'missing_change_mode_service']


def test_relocated_batch_and_corrupt_sidecar(tmp_path):
    campaign = tmp_path / 'campaigns' / 'exp3'
    campaign.mkdir(parents=True)
    attempt(campaign, 'run_0_0', 'completed')
    (campaign / 'run_0_1.done').touch()
    (campaign / 'run_0_1.status.json').write_text('{')
    state = tmp_path / 'state.json'
    state.write_text(json.dumps({'schema_version': 1, 'campaigns': {'exp3': {
        'result_path': '/container/only/exp3', 'expected_runs': 180}}}))
    report = diagnose(state)['campaigns'][0]
    assert report['completed_runs'] == 1
    assert report['expected_runs'] == 180
    assert any('run_0_1.status.json' in item for item in report['warnings'])
    assert any('run_0_1: checkpoint without' in item
               for item in report['warnings'])


def test_unsupported_and_inconsistent_records(tmp_path):
    (tmp_path / 'run_0_0.status.json').write_text(json.dumps({
        'schema_version': 1, 'run_id': 'run_0_0', 'status': 'completed'}))
    record = attempt(tmp_path, 'run_0_1', 'completed')
    (tmp_path / 'run_0_1.done').unlink()
    report = diagnose_campaign(tmp_path)
    assert [r['run_id'] for r in report['runs']] == [record['run_id']]
    assert report['completed_runs'] == 0
    assert any('Incomplete status' in item for item in report['warnings'])
    assert any('disagrees with checkpoint' in item
               for item in report['warnings'])
    state = tmp_path / 'state.json'
    state.write_text(json.dumps({'schema_version': 2, 'campaigns': {}}))
    with pytest.raises(ValueError, match='state-file schema'):
        diagnose(state)


def test_status_sidecar_preserves_attempts_and_markers(tmp_path):
    details = attempt_details('run_0_0', 'bt', 'suave', False)
    path, record = start_status(tmp_path, 0, 0, 'bt', 'suave', **details)
    assert json.loads(path.read_text())['status'] == 'running'
    finish_status(path, record, 'failed', 'missing service', 'readiness')
    details = attempt_details('run_0_0', 'bt', 'suave', False)
    path, record = start_status(tmp_path, 0, 0, 'bt', 'suave', **details)
    assert record['attempts'][0]['status'] == 'failed'
    finish_status(path, record, 'completed')
    assert not (tmp_path / 'run_0_0.done').exists()
    (tmp_path / 'run_0_0.done').touch()
    assert (tmp_path / 'run_0_0.done').read_text() == ''
    assert not list(tmp_path.glob('*.tmp'))
    report = diagnose_campaign(tmp_path)
    assert report['completed_runs'] == 1
    assert report['historical_failed_attempts'] == 1


def test_systemic_requires_repetition():
    failed = {'adaptation_manager': 'bt', 'status': 'failed',
              'failure_type': 'readiness_missing_service'}
    assert summarize([failed])['bt']['systemic'] == []
    assert summarize([failed, failed])['bt']['systemic'] == [
        'readiness_missing_service']
    completed = dict(failed, status='completed')
    assert summarize([failed, failed, completed, completed])[
        'bt']['systemic'] == []
    assert summarize([failed] * 3 + [completed] * 20)['bt']['systemic']
