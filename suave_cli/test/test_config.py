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

"""Tests for suave_cli.config."""

import os

import pytest

from suave_cli import config
from suave_cli.errors import CliError


def test_defaults_come_from_keyspec():
    settings = config.resolve({}, {}, {})
    assert settings.get('exec') == 'auto'
    assert settings.get('container_name') == 'suave'
    assert settings.source('exec') == 'default'
    assert settings.get_bool('mount_src') is True
    assert settings.get('container_workspace') == '/home/ubuntu-user/suave_ws'


def test_precedence_flag_env_file_default():
    file_values = {'exec': 'host', 'container_name': 'from_file', 'image': 'img:file'}
    env = {'SUAVE_EXEC': 'container', 'SUAVE_CONTAINER_NAME': 'from_env'}
    settings = config.resolve({'exec': 'auto'}, env, file_values, file_label='file host.ini')
    assert (settings.get('exec'), settings.source('exec')) == ('auto', 'flag')
    assert (settings.get('container_name'), settings.source('container_name')) == (
        'from_env', 'env SUAVE_CONTAINER_NAME')
    assert (settings.get('image'), settings.source('image')) == ('img:file', 'file host.ini')


def test_none_flags_are_ignored():
    assert config.resolve({'exec': None}, {}, {}).source('exec') == 'default'


def test_bools_are_normalized():
    settings = config.resolve({'mount_src': False}, {}, {'mount_results': 'No'})
    assert settings.get('mount_src') == 'false'
    assert settings.get_bool('mount_results') is False


def test_invalid_values_name_their_source():
    with pytest.raises(CliError, match='exec.*bogus.*env SUAVE_EXEC'):
        config.resolve({}, {'SUAVE_EXEC': 'bogus'}, {})
    with pytest.raises(CliError, match='mount_src.*maybe'):
        config.resolve({}, {}, {'mount_src': 'maybe'})


def test_mount_lists_are_normalized():
    assert config.normalize('extra_mounts', '', 'test') == ''
    value = config.normalize('extra_mounts', '\n/a:/b, /c d:/e\n', 'test')
    assert value == '/a:/b\n/c d:/e'
    assert config.resolve({}, {}, {'extra_mounts': value}).get_list('extra_mounts') == [
        '/a:/b', '/c d:/e']


@pytest.mark.parametrize('entry', ['/a', '/a:/b:ro', 'rel:/b', '/a:rel', ':/b'])
def test_invalid_mount_entries(entry):
    with pytest.raises(CliError, match='extra_mounts'):
        config.normalize('extra_mounts', entry, 'test')


def test_mount_list_round_trips_through_file(tmp_path):
    path = tmp_path / 'host.ini'
    assert config.save_file(path, {'extra_mounts': '/a:/b\n/c:/d'})
    loaded = config.load_file(path)
    assert config.resolve({}, {}, loaded).get_list('extra_mounts') == ['/a:/b', '/c:/d']


def test_unknown_file_key_warns(capsys):
    config.resolve({}, {}, {'colour': 'blue'})
    assert "unknown config key 'colour'" in capsys.readouterr().err


def test_config_path_depends_on_context(tmp_path):
    assert config.config_path(tmp_path, False) == tmp_path / '.config' / 'host.ini'
    assert config.config_path(tmp_path, True) == tmp_path / '.config' / 'container.ini'


def test_save_and_load_roundtrip(tmp_path):
    path = tmp_path / '.config' / 'host.ini'
    assert config.save_file(path, {'exec': 'host', 'image': 'x:100%'})
    assert config.load_file(path) == {'exec': 'host', 'image': 'x:100%'}
    assert path.read_text().startswith('# suave CLI configuration')
    assert [p.name for p in path.parent.iterdir()] == ['host.ini']


def test_load_missing_file_is_empty(tmp_path):
    assert config.load_file(tmp_path / 'nope.ini') == {}


@pytest.mark.skipif(os.geteuid() == 0, reason='root can write read-only dirs')
def test_unwritable_config_warns_and_continues(tmp_path, capsys):
    locked = tmp_path / 'locked'
    locked.mkdir()
    locked.chmod(0o500)
    try:
        assert config.save_file(locked / '.config' / 'host.ini', {'exec': 'host'}) is False
    finally:
        locked.chmod(0o700)
    assert 'cannot write' in capsys.readouterr().err


def test_first_run_non_interactive_only_prints_tip(tmp_path, capsys):
    path = tmp_path / '.config' / 'host.ini'

    def no_input(_prompt):
        raise AssertionError('must not prompt')

    config.maybe_first_run(path, False, interactive=False, input_fn=no_input)
    assert not path.exists()
    assert 'suave config init' in capsys.readouterr().err


def test_first_run_declined_writes_empty_config(tmp_path):
    path = tmp_path / '.config' / 'host.ini'
    config.maybe_first_run(path, False, interactive=True, input_fn=lambda _p: 'n')
    assert path.exists()
    assert config.load_file(path) == {}


def test_first_run_accepted_runs_wizard(tmp_path, capsys):
    path = tmp_path / '.config' / 'host.ini'
    answers = ['y', 'host', '', '', 'bogus', 'interactive', 'no', '', '', '']
    config.maybe_first_run(path, False, interactive=True, input_fn=lambda _p: answers.pop(0))
    assert config.load_file(path) == {
        'exec': 'host', 'run_mode': 'interactive', 'mount_src': 'false'}
    assert answers == []
    assert 'run_mode' in capsys.readouterr().err


def test_container_wizard_asks_only_container_keys(tmp_path):
    prompts = []
    answers = ['', '/ws']

    def fake_input(prompt):
        prompts.append(prompt)
        return answers.pop(0)

    config.run_wizard(tmp_path / 'container.ini', True, input_fn=fake_input)
    assert [p.split(' ')[0] for p in prompts] == ['exec', 'host_workspace']
    assert config.load_file(tmp_path / 'container.ini') == {'host_workspace': '/ws'}


def test_existing_config_skips_first_run(tmp_path):
    path = tmp_path / 'host.ini'
    path.write_text('[suave]\n')
    config.maybe_first_run(path, False, interactive=True, input_fn=lambda _p: 1 / 0)
