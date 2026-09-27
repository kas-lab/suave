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

"""Tests for suave_cli.container."""

import argparse

import pytest

from suave_cli import container, main
from suave_cli.docker_util import RUNNING_FORMAT
from suave_cli.errors import CliError

STATE = ('docker', 'inspect', '-f', RUNNING_FORMAT)
MOUNTS = ('docker', 'inspect', '-f', container.MOUNT_FORMAT)
IMAGES = ('docker', 'images')
SRC_DIR = '/home/ubuntu-user/suave_ws/src/suave'
RESULTS_DIR = '/home/ubuntu-user/suave/results'


def run_args(**overrides):
    values = {'recreate': False, 'src': None, 'yes': False}
    values.update(overrides)
    return argparse.Namespace(**values)


def results_file(tmp_path):
    return {'host_results_dir': str(tmp_path / 'res')}


def test_run_argv_detached():
    argv = container.run_argv('suave', 'img:1', 'detached', [('/h s', '/c')],
                              ['--network', 'host'])
    assert argv == ['docker', 'run', '-d', '--name', 'suave', '--shm-size=512m',
                    '--security-opt', 'seccomp=unconfined', '-v', '/h s:/c',
                    '--network', 'host', 'img:1', 'sleep', 'infinity']


def test_run_argv_interactive():
    argv = container.run_argv('suave', 'img:1', 'interactive', [], [])
    assert argv[2:4] == ['-it', '--rm']
    assert argv[-2:] == ['img:1', 'bash']


def test_default_mounts(make_ctx, suave_root, tmp_path):
    ctx = make_ctx(suave_root, file_values=results_file(tmp_path), args=run_args())
    assert container.build_mounts(ctx) == [
        (str(suave_root.resolve()), SRC_DIR), (str(tmp_path / 'res'), RESULTS_DIR)]


def test_mount_overrides(make_ctx, suave_root, tmp_path):
    other = tmp_path / 'other src'
    other.mkdir()
    ctx = make_ctx(suave_root, flags={'mount_src': False, 'mount_results': False},
                   args=run_args(src=str(other)))
    assert container.build_mounts(ctx) == [(str(other.resolve()), SRC_DIR)]


def test_no_mounts(make_ctx, suave_root):
    ctx = make_ctx(suave_root, flags={'mount_src': False, 'mount_results': False},
                   args=run_args())
    assert container.build_mounts(ctx) == []


def test_run_creates_container(make_ctx, suave_root, fake_runner, tmp_path, capsys):
    fake_runner.responses[STATE] = (1, '')
    fake_runner.responses[IMAGES] = (0, 'latest\n')
    ctx = make_ctx(suave_root, file_values=results_file(tmp_path), args=run_args())
    assert container.cmd_run(ctx) == 0
    [argv] = fake_runner.find('docker', 'run')
    assert argv[2] == '-d'
    assert argv[-3:] == ['suave-headless:latest', 'sleep', 'infinity']
    assert (tmp_path / 'res').is_dir()
    err = capsys.readouterr().err
    assert 'Using image: suave-headless:latest' in err
    assert 'suave build' in err


def test_running_container_is_reused(make_ctx, suave_root, fake_runner, tmp_path, capsys):
    fake_runner.responses[STATE] = (0, 'true\n')
    fake_runner.responses[MOUNTS] = (
        0, f'suave-headless:latest|{suave_root.resolve()}:{SRC_DIR},'
           f'{tmp_path / "res"}:{RESULTS_DIR},')
    ctx = make_ctx(suave_root, file_values=results_file(tmp_path), args=run_args())
    assert container.cmd_run(ctx) == 0
    assert fake_runner.find('docker', 'run') == []
    err = capsys.readouterr().err
    assert 'reusing' in err
    assert 'warning' not in err


def test_mismatched_container_warns(make_ctx, suave_root, fake_runner, tmp_path, capsys):
    fake_runner.responses[STATE] = (0, 'true\n')
    fake_runner.responses[MOUNTS] = (0, 'old:1|')
    ctx = make_ctx(suave_root, file_values=results_file(tmp_path), args=run_args())
    container.cmd_run(ctx)
    assert '--recreate' in capsys.readouterr().err


def test_stopped_container_is_started(make_ctx, suave_root, fake_runner, tmp_path):
    fake_runner.responses[STATE] = (0, 'false\n')
    ctx = make_ctx(suave_root, file_values=results_file(tmp_path), args=run_args())
    assert container.cmd_run(ctx) == 0
    assert fake_runner.find('docker', 'start') == [['docker', 'start', 'suave']]


def test_recreate_removes_first(make_ctx, suave_root, fake_runner, tmp_path):
    fake_runner.responses[STATE] = (0, 'true\n')
    fake_runner.responses[IMAGES] = (0, 'latest\n')
    ctx = make_ctx(suave_root, file_values=results_file(tmp_path),
                   args=run_args(recreate=True))
    assert container.cmd_run(ctx) == 0
    heads = [call[:2] for call in fake_runner.calls]
    assert heads.index(['docker', 'rm']) < heads.index(['docker', 'run'])


def test_shell_requires_running_container(make_ctx, suave_root, fake_runner):
    fake_runner.responses[STATE] = (1, '')
    with pytest.raises(CliError, match='suave docker run'):
        container.cmd_shell(make_ctx(suave_root))


def test_shell_opens_bash(make_ctx, suave_root, fake_runner):
    fake_runner.responses[STATE] = (0, 'true\n')
    assert container.cmd_shell(make_ctx(suave_root)) == 0
    [argv] = fake_runner.find('docker', 'exec')
    assert argv[2:4] == ['-it', 'suave']
    assert argv[-1].endswith('{ exec bash; }')


def test_stop_and_remove(make_ctx, suave_root, fake_runner):
    fake_runner.responses[STATE] = (0, 'true\n')
    assert container.cmd_stop(make_ctx(suave_root, args=argparse.Namespace(rm=True))) == 0
    assert fake_runner.find('docker', 'stop') == [['docker', 'stop', 'suave']]
    assert fake_runner.find('docker', 'rm') == [['docker', 'rm', 'suave']]


def test_status_of_missing_container(make_ctx, suave_root, fake_runner, capsys):
    fake_runner.responses[STATE] = (1, '')
    assert container.cmd_status(make_ctx(suave_root)) == 1
    assert 'does not exist' in capsys.readouterr().out


def test_docker_missing_is_reported(suave_root, fake_runner, capsys):
    fake_runner.responses[('docker',)] = FileNotFoundError()
    code = main.main(['docker', 'status'], env={'SUAVE_ROOT': str(suave_root)},
                     runner=fake_runner)
    assert code == 1
    assert 'Docker is not available' in capsys.readouterr().err
