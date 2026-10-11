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

from suave_runner.diagnose_batch import diagnose
from suave_runner.diagnose_batch import diagnose_campaign
from suave_runner.diagnose_batch import summarize


def test_legacy_resume_and_shutdown_noise(tmp_path):
    campaign = tmp_path / 'campaigns' / 'exp3'
    logs = campaign / 'logs' / 'run_2_0' / 'experiment'
    logs.mkdir(parents=True)
    (tmp_path / 'exp3.log').write_text(
        '[INFO] [1.0] [runner]: Running experiment 3/6 for 30 runs with bt\n'
        '[INFO] [2.0] [runner]:   Run bt 1/30\n'
        '[WARN] [3.0] [runner]: Timeout waiting for mission_metrics/done.\n'
        '[INFO] [4.0] [runner]: Run 1 completed (success: False).\n'
        '[INFO] [5.0] [runner]:   Run bt 1/30\n'
        '[INFO] [6.0] [runner]: Run 1 completed (success: True).\n')
    (logs / 'launch.log').write_text(
        '2.1 [bt] f_maintain_motion/change_mode service not available\n'
        "3.1 [launch] sending signal 'SIGINT'\n"
        '3.2 [mission_metrics] InvalidHandle\n')
    report = diagnose_campaign(campaign)
    assert report['historical_failed_attempts'] == 1
    assert report['completed_runs'] == 1
    assert 'suspected_causes' not in report['runs'][0]


def test_failure_evidence_stops_at_teardown(tmp_path):
    folder = tmp_path / 'logs' / 'run_0_0' / 'experiment'
    folder.mkdir(parents=True)
    (tmp_path / 'run_0_0.status.json').write_text(json.dumps({
        'run_id': 'run_0_0', 'status': 'failed', 'started_epoch': 10.0,
        'failure_type': 'mission_done_timeout', 'adaptation_manager': 'bt',
    }))
    (folder / 'launch.log').write_text(
        '9.0 [mission_metrics] InvalidHandle\n'
        '10.1 [bt] f_maintain_motion/change_mode service not available\n'
        "11.0 [launch] sending signal 'SIGINT'\n"
        '11.1 [mission_metrics] InvalidHandle\n')
    report = diagnose_campaign(tmp_path)
    assert [e['type'] for e in report['runs'][0]['suspected_causes']] == [
        'missing_change_mode_service']


def test_relocated_batch_and_corrupt_sidecar(tmp_path):
    campaign = tmp_path / 'campaigns' / 'exp3'
    campaign.mkdir(parents=True)
    (campaign / 'run_0_0.done').touch()
    (campaign / 'run_0_0.status.json').write_text('{')
    state = tmp_path / 'state.json'
    state.write_text(json.dumps({'campaigns': {'exp3': {
        'result_path': '/container/only/exp3', 'expected_runs': 180}}}))
    report = diagnose(state)['campaigns'][0]
    assert report['completed_runs'] == 1
    assert report['expected_runs'] == 180
    assert report['warnings']


def test_status_sidecar_preserves_attempts_and_markers(tmp_path):
    from suave_runner.run_status import finish_status
    from suave_runner.run_status import start_status

    path, record = start_status(tmp_path, 0, 0, 'bt', 'suave')
    assert json.loads(path.read_text())['status'] == 'running'
    finish_status(path, record, 'failed', 'missing service', 'readiness')
    path, record = start_status(tmp_path, 0, 0, 'bt', 'suave')
    assert record['attempts'][0]['status'] == 'failed'
    finish_status(path, record, 'completed')
    assert not (tmp_path / 'run_0_0.done').exists()
    (tmp_path / 'run_0_0.done').touch()
    assert (tmp_path / 'run_0_0.done').read_text() == ''
    assert not list(tmp_path.glob('*.tmp'))
    assert diagnose_campaign(tmp_path)['completed_runs'] == 1


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
