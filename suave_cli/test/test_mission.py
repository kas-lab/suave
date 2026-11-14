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

"""Tests for suave_cli.mission."""

import argparse
import json
import subprocess

import pytest

from suave_cli import mission
from suave_cli.docker_util import RUNNING_FORMAT
from suave_cli.errors import CliError

RUNNING = ('docker', 'inspect', '-f', RUNNING_FORMAT)


def params_file_of(body):
    """Run body with a fake ros2 that prints the params file and its arguments."""
    script = 'ros2() { cat "$params"; echo "ARGS: $*"; }; ' + body
    result = subprocess.run(['bash', '-c', script], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    text, args = result.stdout.rsplit('ARGS: ', 1)
    return text, args.strip()


def test_experiment_runs_one_mission():
    assert mission.experiment('none') == {
        'experiment_launch': 'ros2 launch suave_bringup mission.launch.py '
                             'adaptation_manager:=none',
        'num_runs': 1, 'adaptation_manager': 'none', 'mission_name': 'suave'}


def test_bt_waits_for_its_services():
    entry = mission.experiment('bt', ['bt_executable:=suave_bt_extended'])
    assert entry['experiment_launch'].endswith(
        'adaptation_manager:=bt bt_executable:=suave_bt_extended')
    assert entry['required_services'] == mission.BT_REQUIRED_SERVICES


def test_body_writes_params_file_and_runs_runner():
    body = mission.mission_body('bt', ["result_filename:=it's"], result_path='/r', gui=True,
                                run_duration=60, params=['experiment_logging:=true'])
    text, args = params_file_of(body)
    assert args == ('run suave_runner suave_runner --ros-args --params-file '
                    f'{args.split()[5]} -p result_path:=/r -p gui:=true '
                    '-p run_duration:=60 -p experiment_logging:=true')
    yaml = pytest.importorskip('yaml')
    [entry] = yaml.safe_load(text)['/suave_runner_node']['ros__parameters']['experiments']
    assert json.loads(entry) == mission.experiment('bt', ["result_filename:=it's"])


def test_body_removes_params_file(tmp_path):
    body = mission.mission_body()
    script = f'ros2() {{ echo "$params" > {tmp_path}/name; }}; ' + body
    subprocess.run(['bash', '-c', script], check=True)
    name = (tmp_path / 'name').read_text().strip()
    assert name.endswith('.yml')
    assert not __import__('os').path.exists(name)


def test_cmd_mission_runs_in_container(make_ctx, suave_root, fake_runner):
    fake_runner.responses[RUNNING] = (0, 'true\n')
    args = argparse.Namespace(manager='random', launch_arg=[], result_path=None, gui=False,
                              run_duration=None, param=['random_seed:=3'])
    assert mission.cmd_mission(make_ctx(suave_root, args=args)) == 0
    [argv] = fake_runner.find('docker', 'exec')
    assert '"adaptation_manager": "random"' in argv[-1]
    assert argv[-1].endswith('--params-file "$params" -p random_seed:=3; }')


def test_cmd_mission_rejects_bad_overrides(make_ctx, suave_root):
    args = argparse.Namespace(manager='none', launch_arg=['gui=true'], result_path=None,
                              gui=False, run_duration=None, param=[])
    with pytest.raises(CliError, match='KEY:=VALUE'):
        mission.cmd_mission(make_ctx(suave_root, args=args))
