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

"""Tests for suave_cli.selftest."""

from suave_cli import selftest


def test_linter_commands_prefer_ament():
    commands = selftest.linter_commands(which=lambda name: f'/usr/bin/{name}',
                                        has_module=lambda _name: True)
    assert commands == [['ament_flake8', '.'], ['ament_pep257', '.']]


def test_linter_commands_fall_back_to_plain_tools():
    commands = selftest.linter_commands(
        which=lambda name: '/usr/bin/flake8' if name == 'flake8' else None,
        has_module=lambda _name: True)
    assert commands[0] == ['flake8', '.']
    assert commands[1][1:3] == ['-m', 'pydocstyle']


def test_missing_linters_warn(capsys):
    assert selftest.linter_commands(which=lambda _n: None, has_module=lambda _m: False) == []
    assert capsys.readouterr().err.count('warning:') == 2


def test_self_test_dry_run(make_ctx, suave_root, capsys):
    assert selftest.cmd_self_test(make_ctx(suave_root, dry_run=True)) == 0
    assert '-m pytest -q test' in capsys.readouterr().out
