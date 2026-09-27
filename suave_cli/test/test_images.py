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

"""Tests for suave_cli.images and suave docker build."""

import argparse

import pytest

from suave_cli import images, main
from suave_cli.errors import CliError

VERSIONS = {'ARDUSUB_COMMIT': 'abc', 'ARDUPILOT_GAZEBO_COMMIT': 'def'}
VERSION_ARGS = ['--build-arg', 'ARDUSUB_COMMIT=abc', '--build-arg', 'ARDUPILOT_GAZEBO_COMMIT=def']
IMAGES = ('docker', 'images')
INSPECT_IMAGE = ('docker', 'image', 'inspect')


def test_load_versions(suave_root):
    assert images.load_versions(suave_root / 'docker' / 'versions.env') == VERSIONS


def test_default_build_is_headless_latest():
    assert images.build_commands(VERSIONS) == [
        ['docker', 'build', '-t', 'suave-headless:latest', *VERSION_ARGS,
         '-f', 'docker/dockerfile-suave-headless', '.']]


def test_custom_tag():
    [command] = images.build_commands(VERSIONS, tag='exp')
    assert command[3] == 'suave-headless:exp'


def test_all_without_tag_keeps_script_tags_for_kasm():
    commands = images.build_commands(VERSIONS, all_images=True)
    assert [c[3] for c in commands] == ['kasm-jammy:dev', 'suave:dev', 'suave-headless:latest']
    assert 'BASE_IMAGE=kasm-jammy:dev' in commands[1]


def test_all_with_tag_uses_it_everywhere():
    commands = images.build_commands(VERSIONS, all_images=True, tag='t1')
    assert [c[3] for c in commands] == ['kasm-jammy:t1', 'suave:t1', 'suave-headless:t1']
    assert 'BASE_IMAGE=kasm-jammy:t1' in commands[1]


def test_no_cache():
    [command] = images.build_commands(VERSIONS, no_cache=True)
    assert command[:3] == ['docker', 'build', '--no-cache']


@pytest.mark.parametrize('tags, expected', [
    (['dev', 'latest', 'x'], ('suave-headless:latest', False)),
    (['x', 'dev'], ('suave-headless:dev', False)),
    (['exp2', 'exp1'], ('suave-headless:exp2', True)),
    ([], None),
])
def test_pick_local_image(tags, expected):
    assert images.pick_local_image(tags) == expected


def test_local_tags_filters_exact_repo_and_none(make_ctx, suave_root, fake_runner):
    fake_runner.responses[IMAGES] = (0, '<none>\nexp\n\n')
    assert images.local_tags(make_ctx(suave_root).executor) == ['exp']
    assert fake_runner.calls[-1] == [
        'docker', 'images', 'suave-headless', '--format', '{{.Tag}}']


def test_ensure_image_prints_choice(make_ctx, suave_root, fake_runner, capsys):
    fake_runner.responses[IMAGES] = (0, 'dev\n')
    assert images.ensure_image(make_ctx(suave_root)) == 'suave-headless:dev'
    assert 'Using image: suave-headless:dev' in capsys.readouterr().err


def test_other_tag_warns(make_ctx, suave_root, fake_runner, capsys):
    fake_runner.responses[IMAGES] = (0, 'exp\n')
    assert images.ensure_image(make_ctx(suave_root)) == 'suave-headless:exp'
    err = capsys.readouterr().err
    assert 'warning:' in err
    assert 'suave-headless:exp' in err


def test_explicit_image_wins(make_ctx, suave_root, fake_runner):
    assert images.ensure_image(make_ctx(suave_root, flags={'image': 'mine:1'})) == 'mine:1'
    assert fake_runner.find(*IMAGES) == []


def test_ghcr_used_when_present(make_ctx, suave_root, fake_runner):
    fake_runner.responses[IMAGES] = (0, '')
    assert images.ensure_image(make_ctx(suave_root)) == images.GHCR_IMAGE
    assert fake_runner.find('docker', 'pull') == []


def test_pull_needs_consent_when_not_interactive(make_ctx, suave_root, fake_runner):
    fake_runner.responses[IMAGES] = (0, '')
    fake_runner.responses[INSPECT_IMAGE] = (1, '')
    with pytest.raises(CliError, match='--yes'):
        images.ensure_image(make_ctx(suave_root))


def test_pull_with_yes(make_ctx, suave_root, fake_runner):
    fake_runner.responses[IMAGES] = (0, '')
    fake_runner.responses[INSPECT_IMAGE] = (1, '')
    ctx = make_ctx(suave_root, args=argparse.Namespace(yes=True))
    assert images.ensure_image(ctx) == images.GHCR_IMAGE
    assert fake_runner.find('docker', 'pull') == [['docker', 'pull', images.GHCR_IMAGE]]


def test_pull_declined_interactively(make_ctx, suave_root, fake_runner):
    fake_runner.responses[IMAGES] = (0, '')
    fake_runner.responses[INSPECT_IMAGE] = (1, '')
    with pytest.raises(CliError, match='not available'):
        images.ensure_image(make_ctx(suave_root, interactive=True, answers=['n']))


def test_docker_build_dry_run(suave_root, capsys):
    assert main.main(['docker', 'build', '--dry-run'], env={'SUAVE_ROOT': str(suave_root)}) == 0
    out = capsys.readouterr().out
    assert 'docker build -t suave-headless:latest --build-arg ARDUSUB_COMMIT=abc' in out
    assert 'kasm' not in out


def test_docker_build_refused_in_container(suave_root, capsys):
    env = {'SUAVE_ROOT': str(suave_root), 'SUAVE_CLI_CONTEXT': 'container'}
    assert main.main(['docker', 'build'], env=env) == 1
    assert 'on the host' in capsys.readouterr().err


def test_docker_build_needs_full_checkout(tmp_path, capsys):
    root = tmp_path / 'r'
    (root / 'suave_cli').mkdir(parents=True)
    assert main.main(['docker', 'build', '--dry-run'], env={'SUAVE_ROOT': str(root)}) == 1
    assert 'cannot build images' in capsys.readouterr().err
