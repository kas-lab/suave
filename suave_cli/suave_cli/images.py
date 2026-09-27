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

"""Select and build SUAVE Docker images."""

import argparse
from pathlib import Path

from suave_cli import term
from suave_cli.context import validate_docker_root
from suave_cli.docker_util import require_host
from suave_cli.errors import CliError

HEADLESS_REPO = 'suave-headless'
GHCR_IMAGE = 'ghcr.io/kas-lab/suave-headless:main'
VERSION_KEYS = ('ARDUSUB_COMMIT', 'ARDUPILOT_GAZEBO_COMMIT')

BUILD_EPILOG = """\
examples:
  suave docker build                   suave-headless:latest
  suave docker build --tag exp1        suave-headless:exp1
  suave docker build --all             also kasm-jammy:dev and suave:dev (GUI)
  suave docker build --all --tag t1    kasm-jammy:t1, suave:t1, suave-headless:t1
"""


def load_versions(path):
    """Parse the KEY=VALUE lines of docker/versions.env."""
    versions = {}
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        line = line.strip()
        if not line or line.startswith('#') or '=' not in line:
            continue
        key, _, value = line.partition('=')
        versions[key.strip()] = value.strip().strip('\'"')
    return versions


def build_commands(versions, all_images=False, tag=None, no_cache=False):
    """Return the docker build command lines in build order."""
    version_args = []
    for key in VERSION_KEYS:
        if key in versions:
            version_args += ['--build-arg', f'{key}={versions[key]}']
    kasm_tag = tag or 'dev'
    headless_tag = tag or 'latest'
    commands = []
    if all_images:
        commands.append(['docker', 'build', '-t', f'kasm-jammy:{kasm_tag}',
                         '-f', 'docker/dockerfile-kasm-core-jammy', '.'])
        commands.append(['docker', 'build', '-t', f'suave:{kasm_tag}',
                         '--build-arg', f'BASE_IMAGE=kasm-jammy:{kasm_tag}', *version_args,
                         '-f', 'docker/dockerfile-suave', '.'])
    commands.append(['docker', 'build', '-t', f'{HEADLESS_REPO}:{headless_tag}', *version_args,
                     '-f', 'docker/dockerfile-suave-headless', '.'])
    if no_cache:
        commands = [command[:2] + ['--no-cache'] + command[2:] for command in commands]
    return commands


def local_tags(executor, repo=HEADLESS_REPO):
    """Return local tags of exactly repo, newest first."""
    code, out = executor.capture(['docker', 'images', repo, '--format', '{{.Tag}}'])
    if code != 0:
        return []
    tags = [line.strip() for line in out.splitlines()]
    return [tag for tag in tags if tag and tag != '<none>']


def pick_local_image(tags):
    """Return (image, is_fallback) for the preferred local tag, or None."""
    for preferred in ('latest', 'dev'):
        if preferred in tags:
            return f'{HEADLESS_REPO}:{preferred}', False
    if tags:
        return f'{HEADLESS_REPO}:{tags[0]}', True
    return None


def image_exists(executor, image):
    """Return True when image is available locally."""
    code, _ = executor.capture(['docker', 'image', 'inspect', image])
    return code == 0


def pull(ctx, image):
    """Pull image after confirmation (or --yes); raise CliError otherwise."""
    if not getattr(ctx.args, 'yes', False):
        if not ctx.interactive:
            raise CliError(f'image {image} is not available locally; pass --yes to pull it')
        question = f'Image {image} is not available locally. Pull it (several GB)?'
        if not term.ask_yes_no(question, input_fn=ctx.input_fn):
            raise CliError(f'image {image} is not available')
    code = ctx.executor.run(['docker', 'pull', image])
    if code:
        raise CliError(f'docker pull {image} failed', code=code)


def ensure_image(ctx):
    """Return the image for suave docker run, pulling the GHCR image if needed."""
    executor = ctx.executor
    explicit = ctx.settings.get('image')
    if explicit:
        if not image_exists(executor, explicit):
            pull(ctx, explicit)
        term.info(f'Using image: {explicit}')
        return explicit
    picked = pick_local_image(local_tags(executor))
    if picked:
        image, fallback = picked
        if fallback:
            term.warn(f'no {HEADLESS_REPO}:latest or :dev found locally; using {image}')
        term.info(f'Using image: {image}')
        return image
    if not image_exists(executor, GHCR_IMAGE):
        pull(ctx, GHCR_IMAGE)
    term.info(f'Using image: {GHCR_IMAGE}')
    return GHCR_IMAGE


def cmd_build(ctx):
    """Build the requested images from SUAVE_ROOT."""
    require_host(ctx)
    validate_docker_root(ctx.suave_root)
    versions = load_versions(ctx.suave_root / 'docker' / 'versions.env')
    args = ctx.args
    for command in build_commands(versions, args.all_images, args.tag, args.no_cache):
        code = ctx.executor.run(command, cwd=ctx.suave_root)
        if code:
            return code
    return 0


def add_parsers(sub, common):
    """Add suave docker build."""
    build = sub.add_parser(
        'build', parents=[common], help='build SUAVE images (default: suave-headless:latest)',
        description='Build SUAVE Docker images from SUAVE_ROOT with the versions pinned in '
                    'docker/versions.env. build_docker_images.sh stays available.',
        epilog=BUILD_EPILOG, formatter_class=argparse.RawDescriptionHelpFormatter)
    build.add_argument('--all', dest='all_images', action='store_true',
                       help='also build the Kasm GUI images (kasm-jammy, suave)')
    build.add_argument('--tag', help='tag for the built images '
                                     '(default: latest for headless, dev for Kasm)')
    build.add_argument('--no-cache', action='store_true', help='pass --no-cache to docker build')
    build.set_defaults(func=cmd_build)
