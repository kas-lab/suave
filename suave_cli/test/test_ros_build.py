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

"""Tests for suave_cli.ros_build."""

import argparse
import os
import subprocess

from suave_cli import ros_build
from suave_cli.docker_util import RUNNING_FORMAT

FAKE_COLCON = """#!/bin/bash
if [ "$1" = test-result ]; then
  case "$*" in *build/bad*) exit 1;; esac
fi
exit 0
"""


def run_body(tmp_path, body, built):
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    colcon = bin_dir / 'colcon'
    colcon.write_text(FAKE_COLCON)
    colcon.chmod(0o755)
    for name in built:
        (tmp_path / 'build' / name).mkdir(parents=True)
    env = {'PATH': f'{bin_dir}:{os.environ["PATH"]}'}
    return subprocess.run(['bash', '-c', body], cwd=tmp_path, env=env).returncode


def test_build_body_default_packages():
    body = ros_build.build_body(ros_build.selected_packages([]))
    assert body.startswith('colcon build --symlink-install --packages-select suave suave_base ')
    assert 'suave_cli' not in body


def test_build_body_clean_and_extra():
    assert ros_build.build_body(['a'], clean=True, extra=['--cmake-args', '-DX=1']) == (
        'rm -rf build/a install/a && '
        'colcon build --symlink-install --packages-select a --cmake-args -DX=1')


def test_test_body_builds_first_and_checks_each_package():
    body = ros_build.test_body(['a', 'b'])
    assert body.startswith('colcon build --symlink-install --packages-select a b || exit $?; ')
    assert 'colcon test --packages-select a b --event-handlers console_direct+ || rc=1' in body
    assert 'colcon test-result --verbose --test-result-base "build/$p"' in body


def test_test_body_lint_and_keyword():
    body = ros_build.test_body(['a'], build=False, lint=True, keyword='flake8')
    assert body.startswith('rc=0; ')
    assert '--pytest-args -m linter -k flake8 --ctest-args -L linter' in body


def test_exit_code_reflects_failed_package(tmp_path):
    body = ros_build.test_body(['ok', 'bad'], build=False)
    assert run_body(tmp_path, body, ['ok', 'bad']) == 1


def test_exit_code_zero_when_all_pass(tmp_path):
    assert run_body(tmp_path, ros_build.test_body(['ok'], build=False), ['ok']) == 0


def test_unbuilt_package_fails(tmp_path):
    assert run_body(tmp_path, ros_build.test_body(['ok', 'ghost'], build=False), ['ok']) == 1


def test_cmd_test_runs_in_container(make_ctx, suave_root, fake_runner):
    fake_runner.responses[('docker', 'inspect', '-f', RUNNING_FORMAT)] = (0, 'true\n')
    args = argparse.Namespace(packages=['suave_runner'], no_build=False, lint=False,
                              keyword=None)
    ctx = make_ctx(suave_root, args=args, passthrough=['--retest-until-pass', '2'])
    assert ros_build.cmd_test(ctx) == 0
    [argv] = fake_runner.find('docker', 'exec')
    assert 'colcon test --packages-select suave_runner' in argv[-1]
    assert '--retest-until-pass 2' in argv[-1]
