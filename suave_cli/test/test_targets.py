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

"""Tests for suave_cli.targets."""

import subprocess

import pytest

from suave_cli.docker_util import RUNNING_FORMAT
from suave_cli.errors import CliError
from suave_cli.targets import run_in_target, select_target, Target, target_results_dir

RUNNING = ('docker', 'inspect', '-f', RUNNING_FORMAT)


def host_files(ros_setup):
    return {'ros_setup': str(ros_setup)}


def test_local_argv_sources_ros_then_workspace():
    target = Target('local', '/my ws', '/opt/ros/humble/setup.bash')
    argv = target.argv('colcon build')
    assert argv[:2] == ['bash', '-lc']
    assert argv[2] == (
        "source /opt/ros/humble/setup.bash && cd '/my ws' && "
        'if [ -f install/setup.bash ]; then source install/setup.bash; fi && '
        '{ colcon build; }')


def test_docker_argv_uses_tty_only_when_asked():
    target = Target('docker', '/home/ubuntu-user/suave_ws', '/opt/ros/humble/setup.bash', 'suave')
    assert target.argv('x')[:4] == ['docker', 'exec', 'suave', 'bash']
    assert target.argv('x', tty=True)[:4] == ['docker', 'exec', '-it', 'suave']


def test_prefix_failure_skips_whole_body(tmp_path):
    marker = tmp_path / 'ran'
    target = Target('local', str(tmp_path), str(tmp_path / 'missing.bash'))
    result = subprocess.run(target.argv(f'true; touch {marker}'), capture_output=True)
    assert result.returncode != 0
    assert not marker.exists()


def test_inside_container_runs_locally(make_ctx, suave_root, ros_setup, fake_runner):
    ctx = make_ctx(suave_root, file_values=host_files(ros_setup), container=True)
    target = select_target(ctx)
    assert (target.kind, target.workspace) == ('local', str(suave_root.parent.parent))
    assert fake_runner.calls == []


def test_auto_prefers_running_container(make_ctx, suave_root, ros_setup, fake_runner, capsys):
    fake_runner.responses[RUNNING] = (0, 'true\n')
    target = select_target(make_ctx(suave_root, file_values=host_files(ros_setup)))
    assert (target.kind, target.container) == ('docker', 'suave')
    assert target.workspace == '/home/ubuntu-user/suave_ws'
    assert "container 'suave'" in capsys.readouterr().err


def test_auto_falls_back_to_host(make_ctx, suave_root, ros_setup, fake_runner):
    fake_runner.responses[RUNNING] = (1, '')
    assert select_target(make_ctx(suave_root, file_values=host_files(ros_setup))).kind == 'local'


def test_auto_without_docker_binary_uses_host(make_ctx, suave_root, ros_setup, fake_runner):
    fake_runner.responses[('docker',)] = FileNotFoundError()
    assert select_target(make_ctx(suave_root, file_values=host_files(ros_setup))).kind == 'local'


def test_auto_with_nothing_available_explains(make_ctx, suave_root, fake_runner, tmp_path):
    fake_runner.responses[RUNNING] = (1, '')
    ctx = make_ctx(suave_root, file_values={'ros_setup': str(tmp_path / 'none.bash')})
    with pytest.raises(CliError, match='suave docker run'):
        select_target(ctx)


def test_container_mode_requires_running_container(make_ctx, suave_root, fake_runner):
    fake_runner.responses[RUNNING] = (1, '')
    with pytest.raises(CliError, match='not running'):
        select_target(make_ctx(suave_root, flags={'exec': 'container'}))


def test_host_mode_never_touches_docker(make_ctx, suave_root, ros_setup, fake_runner):
    ctx = make_ctx(suave_root, file_values=host_files(ros_setup), flags={'exec': 'host'})
    assert select_target(ctx).kind == 'local'
    assert fake_runner.calls == []


def test_target_results_dir(make_ctx, suave_root, tmp_path):
    ctx = make_ctx(suave_root, file_values={'host_results_dir': str(tmp_path / 'r')})
    docker = Target('docker', '/ws', '/s', 'suave')
    assert target_results_dir(ctx, docker) == '/home/ubuntu-user/suave/results'
    assert target_results_dir(ctx, Target('local', '/ws', '/s')) == str(tmp_path / 'r')


def test_run_in_target_executes_argv(make_ctx, suave_root, fake_runner):
    target = Target('docker', '/ws', '/s', 'suave')
    assert run_in_target(make_ctx(suave_root), 'echo hi', target) == 0
    assert fake_runner.find('docker', 'exec')[0][-1].endswith('{ echo hi; }')
