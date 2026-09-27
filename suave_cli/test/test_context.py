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

"""Tests for suave_cli.context."""

import pytest

from suave_cli import context
from suave_cli.context import (
    find_workspace, in_container, resolve_suave_root, validate_docker_root)
from suave_cli.errors import CliError


def test_in_container_uses_explicit_marker():
    assert in_container({'SUAVE_CLI_CONTEXT': 'container'})
    assert not in_container({})
    assert not in_container({'SUAVE_CLI_CONTEXT': 'host'})


def test_root_precedence_flag_env_default(suave_root, tmp_path):
    other = tmp_path / 'other'
    (other / 'suave_cli').mkdir(parents=True)
    env = {'SUAVE_ROOT': str(other)}
    assert resolve_suave_root(str(suave_root), env, default_root=tmp_path) == suave_root.resolve()
    assert resolve_suave_root(None, env, default_root=suave_root) == other.resolve()
    assert resolve_suave_root(None, {}, default_root=suave_root) == suave_root.resolve()


def test_root_without_cli_dir_is_rejected(tmp_path):
    with pytest.raises(CliError, match='not a SUAVE checkout'):
        resolve_suave_root(str(tmp_path), {}, default_root=tmp_path)


def test_default_root_is_this_checkout():
    assert (context.DEFAULT_ROOT / 'suave_cli' / 'suave_cli' / 'context.py').is_file()


def test_find_workspace_uses_src_ancestor(suave_root):
    assert find_workspace(suave_root) == suave_root.parent.parent


def test_find_workspace_ignores_dirs_without_src(tmp_path):
    root = tmp_path / 'a' / 'suave'
    (root / 'suave_cli').mkdir(parents=True)
    (tmp_path / 'install').mkdir()
    assert find_workspace(root) is None


def test_find_workspace_configured_must_exist(suave_root, tmp_path):
    assert find_workspace(suave_root, str(tmp_path)) == tmp_path
    with pytest.raises(CliError, match='does not exist'):
        find_workspace(suave_root, str(tmp_path / 'missing'))


def test_validate_docker_root(suave_root, tmp_path):
    validate_docker_root(suave_root)
    bare = tmp_path / 'bare'
    (bare / 'suave_cli').mkdir(parents=True)
    with pytest.raises(CliError, match='build_docker_images.sh'):
        validate_docker_root(bare)
