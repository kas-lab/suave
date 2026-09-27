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

"""Tests for suave_cli.executor and suave_cli.docker_util."""

import pytest

from suave_cli.docker_util import (
    container_state, docker_available, require_docker, require_host)
from suave_cli.errors import CliError
from suave_cli.executor import Executor, join, Raw


def test_join_quotes_everything_but_raw():
    assert join(['echo', 'a b', Raw('"$(pwd)"/x')]) == "echo 'a b' \"$(pwd)\"/x"


def test_dry_run_prints_and_skips(fake_runner, capsys):
    executor = Executor(dry_run=True, runner=fake_runner)
    assert executor.run(['docker', 'build', '.'], cwd='/r o') == 0
    assert fake_runner.calls == []
    assert capsys.readouterr().out.strip() == "(cd '/r o' && docker build .)"


def test_capture_runs_even_in_dry_run(fake_runner):
    fake_runner.responses[('docker', 'images')] = (0, 'latest\n')
    executor = Executor(dry_run=True, runner=fake_runner)
    assert executor.capture(['docker', 'images']) == (0, 'latest\n')


def test_run_returns_exit_code(fake_runner):
    fake_runner.responses[('false',)] = (3, '')
    assert Executor(runner=fake_runner).run(['false']) == 3


def test_missing_program(fake_runner):
    fake_runner.responses[('docker',)] = FileNotFoundError()
    executor = Executor(runner=fake_runner)
    with pytest.raises(CliError) as info:
        executor.run(['docker', 'ps'])
    assert info.value.code == 127
    assert executor.capture(['docker', 'ps']) == (127, '')


def test_verbose_echoes_commands(fake_runner, capsys):
    Executor(verbose=True, runner=fake_runner).run(['echo', 'hi there'])
    assert "+ echo 'hi there'" in capsys.readouterr().err


def test_docker_available_and_state(fake_runner):
    executor = Executor(runner=fake_runner)
    fake_runner.responses[('docker', 'info')] = (1, '')
    assert docker_available(executor) is False
    fake_runner.responses[('docker', 'inspect')] = (0, 'true\n')
    assert container_state(executor, 'suave') == 'running'
    fake_runner.responses[('docker', 'inspect')] = (0, 'false\n')
    assert container_state(executor, 'suave') == 'stopped'
    fake_runner.responses[('docker', 'inspect')] = (1, '')
    assert container_state(executor, 'suave') is None


def test_require_host_and_docker(make_ctx, suave_root, fake_runner):
    with pytest.raises(CliError, match='on the host'):
        require_host(make_ctx(suave_root, container=True))
    fake_runner.responses[('docker', 'info')] = (1, '')
    with pytest.raises(CliError, match='Docker is not available'):
        require_docker(make_ctx(suave_root))
