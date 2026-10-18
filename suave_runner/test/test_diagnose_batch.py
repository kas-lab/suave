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

"""Regression tests for read-only batch diagnosis and run summaries."""

import json
from unittest.mock import patch

import pytest

from suave_runner.diagnose_batch import campaign_progress
from suave_runner.diagnose_batch import classify
from suave_runner.diagnose_batch import diagnose
from suave_runner.diagnose_batch import diagnose_campaign
from suave_runner.diagnose_batch import summarize
from suave_tools.runs import write_runs


def finished(attempt, campaign, run_id, status, failure_type=''):
    return attempt(campaign, run_id, enabled=False, outcome=status,
                   failure_type=failure_type)[1]


def test_retry_keeps_failed_attempt_history(tmp_path, attempt):
    finished(attempt, tmp_path, 'run_2_0', 'failed', 'mission_done_timeout')
    finished(attempt, tmp_path, 'run_2_0', 'completed')
    report = diagnose_campaign(tmp_path)
    assert report['historical_failed_attempts'] == 1
    assert report['completed_runs'] == 1
    assert report['runs'][0]['run_id'] == 'run_2_0'
    assert 'suspected_causes' not in report['runs'][0]
    assert not report['warnings']


def test_failure_evidence_stops_at_teardown(tmp_path, attempt):
    entry = finished(attempt, tmp_path, 'run_0_0', 'failed',
                     'mission_done_timeout')
    folder = tmp_path / 'logs' / 'run_0_0' / entry['attempt_id'] / 'experiment'
    folder.mkdir(parents=True)
    (folder / 'launch.log').write_text(
        '10.1 [bt] f_maintain_motion/change_mode service not available\n'
        "11.0 [launch] sending signal 'SIGINT'\n"
        '11.1 [mission_metrics] InvalidHandle\n')
    report = diagnose_campaign(tmp_path)
    assert [e['type'] for e in report['runs'][0]['suspected_causes']] == [
        'missing_change_mode_service']


def test_classify_lifecycle_watchdog_report():
    line = ('12.0 [lifecycle_watchdog-14] [ERROR] [12.0] [lifecycle_watchdog]: '
            'f_generate_search_path_node did not become active within 10.0s '
            "of mode_manager's activate request")
    assert classify(line) == 'lost_lifecycle_activation'
    assert classify('f_generate_search_path_node did not become active') == ''


def test_manifest_campaigns_and_corrupt_runs(tmp_path, attempt):
    first = tmp_path / 'campaigns' / 'exp3'
    first.mkdir(parents=True)
    finished(attempt, first, 'run_0_0', 'completed')
    finished(attempt, first, 'run_0_1', 'failed', 'startup_failure')
    broken = tmp_path / 'campaigns' / 'exp4'
    broken.mkdir()
    (broken / 'runs.json').write_text('{')
    config = tmp_path / 'exp3.yml'
    config.write_text(
        '/suave_runner_node:\n  ros__parameters:\n    experiments:\n'
        '      - \'{"num_runs": 2}\'\n      - \'{"num_runs": 3}\'\n')
    state = tmp_path / 'state.json'
    state.write_text(json.dumps({'schema_version': 2, 'campaigns': [
        {'name': 'exp3', 'package': 'pkg', 'config_file': 'exp3.yml'},
        {'name': 'exp4', 'package': 'missing', 'config_file': 'x.yml'}]}))

    def share(package):
        if package != 'pkg':
            raise KeyError(package)
        return str(tmp_path)

    with patch('suave_runner.diagnose_batch.get_package_share_directory',
               side_effect=share):
        exp3, exp4 = diagnose(tmp_path)['campaigns']
    assert (exp3['completed_runs'], exp3['expected_runs']) == (1, 5)
    assert not exp3['warnings']
    assert exp4['expected_runs'] is None and exp4['runs'] == []
    assert any('runs.json' in item for item in exp4['warnings'])
    assert campaign_progress(first, config) == (1, 5)
    assert campaign_progress(tmp_path / 'campaigns' / 'new', config) == (
        0, 5)


def test_unsupported_and_inconsistent_records(tmp_path, attempt):
    runs, entry = attempt(tmp_path, enabled=True, bag=False)
    entry.update(status='completed', bag='pending')
    write_runs(tmp_path, runs)
    report = diagnose_campaign(tmp_path)
    assert report['completed_runs'] == 1
    assert any('run_0_0: completed with bag pending' in item
               for item in report['warnings'])
    state = tmp_path / 'state.json'
    state.write_text(json.dumps({'schema_version': 1, 'campaigns': {}}))
    with pytest.raises(ValueError, match='state-file schema'):
        diagnose(state)
    (tmp_path / 'empty').mkdir()
    with pytest.raises(ValueError, match='expected a batch'):
        diagnose(tmp_path / 'empty')


@pytest.mark.parametrize('relative, names', [
    ('', ['exp3', 'exp1']),
    ('state.json', ['exp3', 'exp1']),
    ('campaigns', ['exp3', 'exp1']),
    ('campaigns/exp1', ['exp1']),
])
def test_accepts_every_batch_form(tmp_path, attempt, relative, names):
    """A batch lists manifest campaigns; one campaign keeps its count."""
    for name in ('exp3', 'exp1'):
        campaign = tmp_path / 'campaigns' / name
        campaign.mkdir(parents=True)
        finished(attempt, campaign, 'run_0_0', 'completed')
    (tmp_path / 'exp.yml').write_text(
        '/suave_runner_node:\n  ros__parameters:\n    experiments:\n'
        '      - \'{"num_runs": 2}\'\n')
    (tmp_path / 'state.json').write_text(json.dumps({
        'schema_version': 2, 'campaigns': [
            {'name': name, 'package': 'pkg', 'config_file': 'exp.yml'}
            for name in ('exp3', 'exp1')]}))
    with patch('suave_runner.diagnose_batch.get_package_share_directory',
               return_value=str(tmp_path)):
        report = diagnose(tmp_path / relative)
    assert [c['path'].rsplit('/', 1)[1] for c in report['campaigns']] == names
    assert {c['expected_runs'] for c in report['campaigns']} == {2}


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
