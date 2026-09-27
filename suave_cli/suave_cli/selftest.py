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

"""suave self-test: run the CLI's own tests and linters without colcon."""

import importlib.util
import os
import shutil
import sys

from suave_cli import term

PYDOCSTYLE_IGNORE = 'D100,D101,D102,D103,D104,D105,D106,D107,D404'


def _has_module(name):
    return importlib.util.find_spec(name) is not None


def linter_commands(which=shutil.which, has_module=None):
    """Return the available flake8 and pep257 commands, preferring the ament ones."""
    has_module = has_module or _has_module
    commands = []
    if which('ament_flake8'):
        commands.append(['ament_flake8', '.'])
    elif which('flake8'):
        commands.append(['flake8', '.'])
    else:
        term.warn('flake8 not found; skipping it')
    if which('ament_pep257'):
        commands.append(['ament_pep257', '.'])
    elif has_module('pydocstyle'):
        commands.append([sys.executable, '-m', 'pydocstyle', '--convention=pep257',
                         f'--add-ignore={PYDOCSTYLE_IGNORE}', '.'])
    else:
        term.warn('ament_pep257 and pydocstyle not found; skipping docstring checks')
    return commands


def cmd_self_test(ctx):
    """Run the unit tests and linters of suave_cli on this machine."""
    cli_dir = ctx.suave_root / 'suave_cli'
    env = dict(os.environ, PYTEST_DISABLE_PLUGIN_AUTOLOAD='1')
    failed = ctx.executor.run([sys.executable, '-m', 'pytest', '-q', 'test'], cwd=cli_dir,
                              env=env) != 0
    for command in linter_commands():
        failed = ctx.executor.run(command, cwd=cli_dir) != 0 or failed
    return 1 if failed else 0


def register(subparsers, common):
    """Add suave self-test."""
    parser = subparsers.add_parser(
        'self-test', parents=[common], help="run the CLI's own tests and linters",
        description='Run the suave_cli unit tests and linters locally (no colcon needed).')
    parser.set_defaults(func=cmd_self_test, skip_first_run=True)
