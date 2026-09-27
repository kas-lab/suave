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

"""Locate the SUAVE checkout and its workspace; hold per-invocation state."""

from dataclasses import dataclass
from pathlib import Path

from suave_cli.errors import CliError

CONTEXT_ENV = 'SUAVE_CLI_CONTEXT'
DEFAULT_ROOT = Path(__file__).resolve().parents[2]
DOCKER_ROOT_FILES = ('build_docker_images.sh', 'docker/versions.env')


def in_container(env):
    """Return True when the Dockerfile marker says we run in a SUAVE container."""
    return env.get(CONTEXT_ENV) == 'container'


def resolve_suave_root(flag, env, default_root=DEFAULT_ROOT):
    """Return the checkout from --suave-root, $SUAVE_ROOT or the launcher location."""
    if flag:
        candidate, source = Path(flag).expanduser(), '--suave-root'
    elif env.get('SUAVE_ROOT'):
        candidate, source = Path(env['SUAVE_ROOT']).expanduser(), 'SUAVE_ROOT'
    else:
        candidate, source = Path(default_root), 'the launcher location'
    candidate = candidate.resolve()
    if not (candidate / 'suave_cli').is_dir():
        raise CliError(
            f'{candidate} (from {source}) is not a SUAVE checkout: '
            'it has no suave_cli/ directory')
    return candidate


def validate_docker_root(root):
    """Raise CliError unless root is a full checkout that can build images."""
    missing = [name for name in DOCKER_ROOT_FILES if not (Path(root) / name).exists()]
    if missing:
        raise CliError(
            f'{root} cannot build images (missing {", ".join(missing)}); '
            'pass --suave-root to point at a full SUAVE checkout')


def find_workspace(suave_root, configured=''):
    """Return the colcon workspace whose src/ contains suave_root, or None."""
    if configured:
        path = Path(configured).expanduser()
        if not path.is_dir():
            raise CliError(f'host_workspace {path} does not exist')
        return path
    root = Path(suave_root)
    for parent in root.parents:
        src = parent / 'src'
        if src.is_dir() and _is_within(root, src):
            return parent
    return None


def _is_within(path, base):
    try:
        path.relative_to(base)
    except ValueError:
        return False
    return True


@dataclass
class AppContext:
    """Everything a command needs for one invocation."""

    suave_root: Path
    in_container: bool
    interactive: bool
    settings: object
    executor: object
    args: object
    passthrough: list
    config_path: Path
    input_fn: object = input
