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

"""Tests for suave_cli.main, the launcher and env.sh."""

import argparse
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys

import pytest

from suave_cli import main as cli

REPO = Path(__file__).resolve().parents[2]
LAUNCHER = REPO / 'suave_cli' / 'bin' / 'suave'
ENV_SH = REPO / 'env.sh'


@pytest.fixture
def env(suave_root):
    return {'SUAVE_ROOT': str(suave_root)}


def no_input(_prompt):
    raise AssertionError('unexpected prompt')


def test_no_arguments_prints_help(env, capsys):
    assert cli.main([], env=env) == 0
    assert 'config' in capsys.readouterr().out


def test_help_lists_examples(capsys):
    with pytest.raises(SystemExit) as info:
        cli.main(['--help'], env={})
    assert info.value.code == 0
    out = capsys.readouterr().out
    assert 'SUAVE command-line helper' in out
    assert 'examples:' in out


def test_config_set_show_unset(env, suave_root, capsys):
    assert cli.main(['config', 'set', 'exec', 'host'], env=env, input_fn=no_input) == 0
    assert (suave_root / '.config' / 'host.ini').is_file()
    assert cli.main(['config', 'show'], env=env, input_fn=no_input) == 0
    assert re.search(r'exec\s+= host\s+\(file host\.ini\)', capsys.readouterr().out)
    assert cli.main(['config', 'unset', 'exec'], env=env) == 0
    cli.main(['config', 'show'], env=env)
    assert re.search(r'exec\s+= auto\s+\(default\)', capsys.readouterr().out)


def test_global_flags_before_or_after_subcommand(env, capsys):
    cli.main(['--exec', 'host', 'config', 'show'], env=env)
    cli.main(['config', 'show', '--exec', 'container'], env=env)
    out = capsys.readouterr().out
    assert re.search(r'exec\s+= host\s+\(flag\)', out)
    assert re.search(r'exec\s+= container\s+\(flag\)', out)


def test_env_var_is_reported_as_source(suave_root, capsys):
    cli.main(['config', 'show'], env={'SUAVE_ROOT': str(suave_root), 'SUAVE_EXEC': 'host'})
    assert '(env SUAVE_EXEC)' in capsys.readouterr().out


def test_invalid_set_value_fails(env, capsys):
    assert cli.main(['config', 'set', 'exec', 'bogus'], env=env) == 1
    assert 'expected one of' in capsys.readouterr().err


def test_bad_file_can_still_be_fixed(env, suave_root):
    path = suave_root / '.config' / 'host.ini'
    path.parent.mkdir()
    path.write_text('[suave]\nexec = bogus\n')
    assert cli.main(['config', 'show'], env=env) == 1
    assert cli.main(['config', 'set', 'exec', 'host'], env=env) == 0
    assert cli.main(['config', 'show'], env=env) == 0


def test_passthrough_rejected_where_unsupported(env):
    with pytest.raises(SystemExit) as info:
        cli.main(['config', 'show', '--', 'x'], env=env)
    assert info.value.code == 2


def test_first_run_is_prompt_free_when_not_interactive(env, suave_root, capsys):
    ctx = cli.make_context(argparse.Namespace(), env, [], input_fn=no_input)
    assert ctx.settings.get('exec') == 'auto'
    assert not (suave_root / '.config' / 'host.ini').exists()
    assert 'suave config init' in capsys.readouterr().err


def test_container_context_uses_container_config(suave_root):
    env = {'SUAVE_ROOT': str(suave_root), 'SUAVE_CLI_CONTEXT': 'container'}
    ctx = cli.make_context(argparse.Namespace(), env, [], input_fn=no_input)
    assert ctx.in_container
    assert ctx.config_path.name == 'container.ini'


def test_bad_root_is_reported(tmp_path, capsys):
    assert cli.main(['config', 'show'], env={'SUAVE_ROOT': str(tmp_path)}) == 1
    assert 'not a SUAVE checkout' in capsys.readouterr().err


def test_launcher_symlink_finds_checkout(tmp_path):
    link = tmp_path / 'suave'
    link.symlink_to(LAUNCHER)
    clean_env = {'PATH': os.environ['PATH'], 'HOME': str(tmp_path)}
    result = subprocess.run([str(link), 'config', 'path'], capture_output=True, text=True,
                            env=clean_env)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == str(REPO / '.config' / 'host.ini')


def test_python_dash_m_entry():
    result = subprocess.run([sys.executable, '-m', 'suave_cli', '--help'],
                            cwd=REPO / 'suave_cli', capture_output=True, text=True,
                            env={'PATH': os.environ['PATH']})
    assert result.returncode == 0
    assert 'usage: suave' in result.stdout


def test_env_sh_sets_root_and_path_once():
    quoted = shlex.quote(str(ENV_SH))
    script = f'source {quoted}; source {quoted}; echo "$SUAVE_ROOT"; echo "$PATH"'
    result = subprocess.run(['bash', '-c', script], capture_output=True, text=True,
                            env={'PATH': '/usr/bin:/bin'}, check=True)
    root_line, path_line = result.stdout.splitlines()
    assert root_line == str(REPO)
    assert path_line.split(':').count(str(REPO / 'suave_cli' / 'bin')) == 1


def test_env_sh_refuses_execution():
    result = subprocess.run(['bash', str(ENV_SH)], capture_output=True, text=True)
    assert result.returncode == 1
    assert 'source this file' in result.stderr
