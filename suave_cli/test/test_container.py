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

from suave_cli import config, container, display, main
from suave_cli.docker_util import RUNNING_FORMAT
from suave_cli.errors import CliError

STATE = ('docker', 'inspect', '-f', RUNNING_FORMAT)
MOUNTS = ('docker', 'inspect', '-f', container.MOUNT_FORMAT)
IMAGES = ('docker', 'images')
RUNTIMES = ('docker', 'info', '-f', display.RUNTIMES_FORMAT)
XAUTH = ('xauth', 'nlist')
# Mounts added by the default gpu=nvidia setting with DISPLAY unset.
SYSTEM_MOUNTS = '/dev/dri:/dev/dri,/etc/localtime:/etc/localtime,'
SRC_DIR = '/home/ubuntu-user/suave_ws/src/suave'
RESULTS_DIR = '/home/ubuntu-user/suave/results'


WS_SRC = '/home/ubuntu-user/suave_ws/src'


def run_args(**overrides):
    values = {'recreate': False, 'src': None, 'yes': False, 'extra_mount': None,
              'no_extra_mounts': False}
    values.update(overrides)
    return argparse.Namespace(**values)


def results_file(tmp_path):
    return {'host_results_dir': str(tmp_path / 'res')}


def test_run_argv_detached():
    argv = container.run_argv('suave', 'img:1', 'detached', [('/h s', '/c')],
                              ['--network', 'host'])
    assert argv == ['docker', 'run', '-d', '--name', 'suave', '-v', '/h s:/c',
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
    fake_runner.responses[RUNTIMES] = (0, 'nvidia runc ')
    ctx = make_ctx(suave_root, file_values=results_file(tmp_path), args=run_args())
    assert container.cmd_run(ctx) == 0
    [argv] = fake_runner.find('docker', 'run')
    assert argv[2] == '-d'
    assert '--shm-size=512m' not in argv
    gpus = argv.index('--gpus')
    assert argv[gpus:gpus + 3] == ['--gpus', 'all', '--runtime=nvidia']
    assert '/etc/localtime:/etc/localtime:ro' in argv
    assert argv[-3:] == ['suave-headless:latest', 'sleep', 'infinity']
    assert (tmp_path / 'res').is_dir()
    err = capsys.readouterr().err
    assert 'Using image: suave-headless:latest' in err
    assert 'suave build' in err


def test_running_container_is_reused(make_ctx, suave_root, fake_runner, tmp_path, capsys):
    fake_runner.responses[STATE] = (0, 'true\n')
    fake_runner.responses[MOUNTS] = (
        0, f'suave-headless:latest|{suave_root.resolve()}:{SRC_DIR},'
           f'{tmp_path / "res"}:{RESULTS_DIR},{SYSTEM_MOUNTS}')
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
    fake_runner.responses[RUNTIMES] = (0, 'nvidia runc ')
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


def no_defaults():
    return {'mount_src': False, 'mount_results': False}


def test_extra_mounts_follow_defaults(make_ctx, suave_root, tmp_path):
    saved, once = tmp_path / 'saved', tmp_path / 'once'
    saved.mkdir()
    once.mkdir()
    file_values = {**results_file(tmp_path), 'extra_mounts': f'{saved}:/data'}
    ctx = make_ctx(suave_root, file_values=file_values,
                   args=run_args(extra_mount=[str(once), f'{once}:/opt/once']))
    assert container.build_mounts(ctx)[2:] == [
        (str(saved), '/data'), (str(once), f'{WS_SRC}/once'), (str(once), '/opt/once')]


def test_no_extra_mounts_skips_saved_ones(make_ctx, suave_root, tmp_path):
    ctx = make_ctx(suave_root, flags=no_defaults(), file_values={'extra_mounts': '/x:/data'},
                   args=run_args(no_extra_mounts=True))
    assert container.build_mounts(ctx) == []


@pytest.mark.parametrize('dest', [SRC_DIR, f'{SRC_DIR}/sub', WS_SRC, RESULTS_DIR])
def test_overlapping_destinations_are_rejected(make_ctx, suave_root, tmp_path, dest):
    ctx = make_ctx(suave_root, file_values={**results_file(tmp_path),
                                            'extra_mounts': f'{tmp_path}:{dest}'},
                   args=run_args())
    with pytest.raises(CliError, match='overlaps'):
        container.cmd_run(ctx)


def test_nesting_is_allowed_without_the_source_mount(make_ctx, suave_root, tmp_path):
    defaults = container.default_mounts(make_ctx(suave_root, flags=no_defaults()))
    container.check_extra_mounts(defaults, [(str(tmp_path), f'{SRC_DIR}/sub')])


def test_duplicate_extra_destinations_are_rejected(tmp_path):
    with pytest.raises(CliError, match='overlaps'):
        container.check_extra_mounts([], [(str(tmp_path), '/data'), (str(tmp_path), '/data')])


def test_missing_mount_source_is_rejected(make_ctx, suave_root, tmp_path):
    ctx = make_ctx(suave_root, flags=no_defaults(),
                   file_values={'extra_mounts': f'{tmp_path}/gone:/data'}, args=run_args())
    with pytest.raises(CliError, match='does not exist'):
        container.cmd_run(ctx)


def test_matching_extra_mounts_do_not_warn(make_ctx, suave_root, fake_runner, tmp_path, capsys):
    fake_runner.responses[STATE] = (0, 'true\n')
    fake_runner.responses[MOUNTS] = (0, f'img|{tmp_path}:/data,{SYSTEM_MOUNTS}')
    ctx = make_ctx(suave_root, flags=no_defaults(),
                   file_values={'extra_mounts': f'{tmp_path}:/data'}, args=run_args())
    assert container.cmd_run(ctx) == 0
    assert 'recreate' not in capsys.readouterr().err


def cli(suave_root, fake_runner, *argv):
    return main.main(list(argv), env={'SUAVE_ROOT': str(suave_root)}, runner=fake_runner)


def saved(suave_root):
    return config.split_list(
        config.load_file(config.config_path(suave_root, False)).get('extra_mounts', ''))


def test_mount_add_list_remove(suave_root, fake_runner, tmp_path, capsys):
    pkg = tmp_path / 'my_package'
    pkg.mkdir()
    fake_runner.responses[STATE] = (1, '')
    assert cli(suave_root, fake_runner, 'docker', 'mount', 'add', str(pkg)) == 0
    assert cli(suave_root, fake_runner, 'docker', 'mount', 'add', str(pkg), '--to', '/d') == 0
    assert cli(suave_root, fake_runner, 'docker', 'mount', 'add', str(pkg), '--to', '/d') == 0
    assert saved(suave_root) == [f'{pkg}:{WS_SRC}/my_package', f'{pkg}:/d']
    assert 'already mounted' in capsys.readouterr().err
    assert cli(suave_root, fake_runner, 'docker', 'mount', 'list') == 0
    assert f'{pkg} -> /d  (extra_mounts)' in capsys.readouterr().out
    assert cli(suave_root, fake_runner, 'docker', 'mount', 'remove', '/d') == 0
    assert saved(suave_root) == [f'{pkg}:{WS_SRC}/my_package']
    assert cli(suave_root, fake_runner, 'docker', 'mount', 'remove', str(pkg)) == 0
    assert saved(suave_root) == []
    assert cli(suave_root, fake_runner, 'docker', 'mount', 'remove', str(pkg)) == 1


def test_mount_add_rejects_overlap(suave_root, fake_runner, tmp_path, capsys):
    code = cli(suave_root, fake_runner, 'docker', 'mount', 'add', str(tmp_path), '--to', SRC_DIR)
    assert code == 1
    assert 'overlaps' in capsys.readouterr().err
    assert saved(suave_root) == []


def test_mount_add_hints_recreate(suave_root, fake_runner, tmp_path, capsys):
    fake_runner.responses[STATE] = (0, 'true\n')
    assert cli(suave_root, fake_runner, 'docker', 'mount', 'add', str(tmp_path)) == 0
    assert '--recreate' in capsys.readouterr().err


def test_missing_mount_source_does_not_break_other_commands(suave_root, fake_runner, tmp_path):
    pkg = tmp_path / 'pkg'
    pkg.mkdir()
    assert cli(suave_root, fake_runner, 'docker', 'mount', 'add', str(pkg)) == 0
    pkg.rmdir()
    assert cli(suave_root, fake_runner, 'config', 'show') == 0
    assert cli(suave_root, fake_runner, 'docker', 'mount', 'list') == 0


def test_mount_remove_fixes_a_bad_entry(suave_root, fake_runner):
    config.save_file(config.config_path(suave_root, False), {'extra_mounts': 'broken'})
    assert cli(suave_root, fake_runner, 'docker', 'mount', 'remove', 'broken') == 0
    assert saved(suave_root) == []


def create_ctx(make_ctx, suave_root, fake_runner, **kwargs):
    fake_runner.responses[STATE] = (1, '')
    fake_runner.responses[IMAGES] = (0, 'latest\n')
    fake_runner.responses[RUNTIMES] = (0, 'nvidia runc ')
    return make_ctx(suave_root, flags=no_defaults(), args=run_args(), **kwargs)


def test_display_uses_private_cookie(make_ctx, suave_root, fake_runner, tmp_path, monkeypatch):
    monkeypatch.setenv('DISPLAY', ':1')
    monkeypatch.setattr(display.os, 'getuid', lambda: display.CONTAINER_UID)
    fake_runner.responses[XAUTH] = (0, '0100 0004 686f7374 0001 31 0002 4d41 0002 abcd\n')
    ctx = create_ctx(make_ctx, suave_root, fake_runner)
    assert container.cmd_run(ctx) == 0
    [argv] = fake_runner.find('docker', 'run')
    folder = tmp_path / 'cache' / 'suave' / 'xauth' / 'suave'
    expected = ('DISPLAY=:1', 'QT_X11_NO_MITSHM=1', '/tmp/.X11-unix:/tmp/.X11-unix',
                f'XAUTHORITY={display.CONTAINER_XAUTH_DIR}/{display.XAUTH_FILE}',
                f'{folder}:{display.CONTAINER_XAUTH_DIR}:ro')
    assert all(arg in argv for arg in expected)
    cookie = folder / display.XAUTH_FILE
    assert cookie.read_bytes() == bytes.fromhex('ffff0004686f737400013100024d410002abcd')
    assert cookie.stat().st_mode & 0o777 == 0o600
    assert folder.stat().st_mode & 0o777 == 0o700
    assert not any('xhost' in call for call in fake_runner.calls)


def test_gpu_none_skips_nvidia(make_ctx, suave_root, fake_runner):
    ctx = create_ctx(make_ctx, suave_root, fake_runner, env={'SUAVE_GPU': 'none'})
    assert container.cmd_run(ctx) == 0
    [argv] = fake_runner.find('docker', 'run')
    assert '--gpus' not in argv
    assert '/dev/dri:/dev/dri' not in argv
    assert '/etc/localtime:/etc/localtime:ro' in argv
    assert fake_runner.find(*RUNTIMES) == []


def test_missing_nvidia_runtime_is_explained(make_ctx, suave_root, fake_runner):
    ctx = create_ctx(make_ctx, suave_root, fake_runner)
    fake_runner.responses[RUNTIMES] = (0, 'runc ')
    with pytest.raises(CliError, match='--gpu none'):
        container.cmd_run(ctx)


def test_dry_run_writes_no_cookie(make_ctx, suave_root, fake_runner, tmp_path, monkeypatch):
    monkeypatch.setenv('DISPLAY', ':1')
    ctx = create_ctx(make_ctx, suave_root, fake_runner, dry_run=True)
    assert container.cmd_run(ctx) == 0
    assert not (tmp_path / 'cache' / 'suave').exists()


def test_reuse_warns_about_another_display(make_ctx, suave_root, fake_runner, monkeypatch,
                                           capsys):
    monkeypatch.setenv('DISPLAY', ':1')
    fake_runner.responses[STATE] = (0, 'true\n')
    fake_runner.responses[('docker', 'inspect', '-f', container.ENV_FORMAT)] = (
        0, 'PATH=/usr/bin\nDISPLAY=:0\n')
    ctx = make_ctx(suave_root, flags=no_defaults(), args=run_args())
    assert container.cmd_run(ctx) == 0
    assert 'DISPLAY=:0' in capsys.readouterr().err


@pytest.mark.parametrize('dest', ['/tmp', '/dev'])
def test_extra_mounts_cannot_cover_system_mounts(make_ctx, suave_root, tmp_path, monkeypatch,
                                                 dest):
    monkeypatch.setenv('DISPLAY', ':1')
    ctx = make_ctx(suave_root, flags=no_defaults(),
                   file_values={'extra_mounts': f'{tmp_path}:{dest}'}, args=run_args())
    with pytest.raises(CliError, match='overlaps'):
        container.cmd_run(ctx)


def test_wild_cookies_skip_malformed_lines():
    assert display.wild_cookies('\n0100 zz\n0100 0001 41\n') == bytes.fromhex('ffff000141')
