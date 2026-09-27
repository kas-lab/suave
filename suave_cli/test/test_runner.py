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

"""Tests for suave_cli.runner."""

import argparse
import json
import subprocess

import pytest

from suave_cli import runner
from suave_cli.docker_util import RUNNING_FORMAT
from suave_cli.errors import CliError

RUNNING = ('docker', 'inspect', '-f', RUNNING_FORMAT)
INSTALLED = '"$(ros2 pkg prefix --share suave_runner)/config/runner/'


def sh(body):
    return subprocess.run(['bash', '-c', body], capture_output=True, text=True).stdout.strip()


def test_run_without_options_uses_launch_file():
    assert runner.run_body() == 'ros2 launch suave_runner suave_runner_launch.py'


def test_run_overrides_use_installed_config():
    body = runner.run_body(result_path='/r', gui=True, seed=7, run_duration=60,
                           params=['experiment_logging:=true'])
    assert body == (
        'ros2 run suave_runner suave_runner --ros-args --params-file '
        f'{INSTALLED}runner_config.yml" '
        '-p result_path:=/r -p gui:=true -p random_seed:=7 -p run_duration:=60 '
        '-p experiment_logging:=true')


def test_run_with_config_checks_it_exists():
    body = runner.run_body(config_file='my cfg.yml')
    assert body.startswith("if [ ! -e 'my cfg.yml' ]; then")
    assert body.endswith("--params-file 'my cfg.yml'")


def test_batch_start_bodies():
    assert runner.batch_start_body() == 'ros2 launch suave_runner run_batch_launch.py'
    body = runner.batch_start_body(batch_dir='/b', fail_fast=True, dry_run_batch=True)
    assert body.endswith(
        'batch_campaigns.yml" -p batch_dir:=/b -p fail_fast:=true -p dry_run:=true')


def test_batch_resume_body():
    body = runner.batch_resume_body('/b/state.json')
    assert body.startswith('if [ ! -e /b/state.json ]; then')
    assert body.endswith(
        'ros2 run suave_runner run_batch --ros-args -p resume_state_file:=/b/state.json')


def test_campaign_resume_body_default_config():
    body = runner.campaign_resume_body('/r/2026_01_01_10-00-00')
    assert f'--params-file {INSTALLED}runner_config.yml"' in body
    assert body.endswith('-p resume_result_path:=/r/2026_01_01_10-00-00')


def test_latest_body_finds_newest(tmp_path):
    for name in ('batch_20260101_100000', 'batch_20260301_090000'):
        (tmp_path / 'batches' / name).mkdir(parents=True)
        (tmp_path / 'batches' / name / 'state.json').write_text('{}')
    for name in ('2026_01_01_10-00-00', '2026_02_01_10-00-00', 'notes'):
        (tmp_path / name).mkdir()
    newest_batch = tmp_path / 'batches' / 'batch_20260301_090000' / 'state.json'
    assert sh(runner.latest_body('batch', str(tmp_path))) == str(newest_batch)
    assert sh(runner.latest_body('campaign', str(tmp_path))) == str(
        tmp_path / '2026_02_01_10-00-00')


def test_list_bodies(tmp_path):
    state = {'campaign_order': ['a', 'b'],
             'campaigns': {'a': {'status': 'completed'}, 'b': {'status': 'pending'}}}
    batch = tmp_path / 'batches' / 'batch_20260101_100000'
    batch.mkdir(parents=True)
    (batch / 'state.json').write_text(json.dumps(state))
    campaign = tmp_path / '2026_01_01_10-00-00'
    campaign.mkdir()
    (campaign / 'run_0_0.done').write_text('')
    (campaign / 'run_0_1.done').write_text('')
    assert sh(runner.batch_list_body(str(tmp_path))) == (
        f'{batch / "state.json"}  1/2 campaigns completed')
    assert sh(runner.campaign_list_body(str(tmp_path))) == f'{campaign}  2 runs done'


def resume_args(**overrides):
    values = {'state_file': None, 'result_path': None, 'latest': False, 'config': None}
    values.update(overrides)
    return argparse.Namespace(**values)


def test_batch_resume_latest(make_ctx, suave_root, fake_runner, capsys):
    fake_runner.responses[RUNNING] = (0, 'true\n')
    state = '/home/ubuntu-user/suave/results/batches/batch_1/state.json'
    fake_runner.responses[('docker', 'exec')] = (0, f'{state}\n')
    ctx = make_ctx(suave_root, args=resume_args(latest=True))
    assert runner.cmd_batch_resume(ctx) == 0
    execs = fake_runner.find('docker', 'exec')
    assert '/home/ubuntu-user/suave/results' in execs[0][-1]
    assert execs[1][-1].endswith(f'resume_state_file:={state}; }}')
    assert f'Latest batch: {state}' in capsys.readouterr().err


def test_latest_with_nothing_found(make_ctx, suave_root, fake_runner):
    fake_runner.responses[RUNNING] = (0, 'true\n')
    fake_runner.responses[('docker', 'exec')] = (0, '')
    with pytest.raises(CliError, match='no batch found'):
        runner.cmd_batch_resume(make_ctx(suave_root, args=resume_args(latest=True)))


def test_path_and_latest_are_exclusive(make_ctx, suave_root, fake_runner):
    fake_runner.responses[RUNNING] = (0, 'true\n')
    with pytest.raises(CliError, match='either'):
        runner.cmd_batch_resume(
            make_ctx(suave_root, args=resume_args(state_file='/s.json', latest=True)))
    with pytest.raises(CliError, match='either'):
        runner.cmd_campaign_resume(make_ctx(suave_root, args=resume_args()))


def test_campaign_resume_without_config_warns(make_ctx, suave_root, fake_runner, capsys):
    fake_runner.responses[RUNNING] = (0, 'true\n')
    ctx = make_ctx(suave_root, args=resume_args(result_path='/r/2026_01_01_10-00-00'))
    assert runner.cmd_campaign_resume(ctx) == 0
    assert 'must match' in capsys.readouterr().err
