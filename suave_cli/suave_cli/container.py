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

"""Start, reuse, enter, stop and inspect the SUAVE container."""

import argparse
from pathlib import Path

from suave_cli import term
from suave_cli.docker_util import container_state, require_docker
from suave_cli.errors import CliError
from suave_cli.images import ensure_image
from suave_cli.targets import CONTAINER_ROS_SETUP, Target

SHM_SIZE = '512m'
MOUNT_FORMAT = '{{.Config.Image}}|{{range .Mounts}}{{.Source}}:{{.Destination}},{{end}}'

RUN_DESCRIPTION = """\
Start the SUAVE container, or reuse it when it already exists.

Image selection (unless --image or the 'image' setting is given):
  1. suave-headless:latest   2. suave-headless:dev
  3. any other local suave-headless tag (printed as a warning)
  4. ghcr.io/kas-lab/suave-headless:main (pulled after confirmation)

By default this checkout is mounted at the container's src/suave and
~/suave/results at the container's results folder.
"""

RUN_EPILOG = """\
examples:
  suave docker run                              detached, default mounts
  suave docker run --interactive --no-mount-src throwaway shell on the baked-in code
  suave docker run --image suave-headless:exp1 --recreate
  suave docker run -- --network host            extra docker run arguments
"""


def build_mounts(ctx):
    """Return the (host path, container path) bind mounts for docker run."""
    settings, args = ctx.settings, ctx.args
    mounts = []
    src = getattr(args, 'src', None)
    if src or settings.get_bool('mount_src'):
        source = Path(src or ctx.suave_root).expanduser().resolve()
        mounts.append((str(source), settings.get('container_src_dir')))
    if settings.get_bool('mount_results'):
        results = Path(settings.get('host_results_dir')).expanduser()
        mounts.append((str(results), settings.get('container_results_dir')))
    return mounts


def run_argv(name, image, mode, mounts, extra):
    """Return the docker run command line."""
    argv = ['docker', 'run']
    argv += ['-d'] if mode == 'detached' else ['-it', '--rm']
    argv += ['--name', name, f'--shm-size={SHM_SIZE}', '--security-opt', 'seccomp=unconfined']
    for host_path, container_path in mounts:
        argv += ['-v', f'{host_path}:{container_path}']
    argv += list(extra)
    argv += [image, 'sleep', 'infinity'] if mode == 'detached' else [image, 'bash']
    return argv


def _warn_if_different(ctx, name, mounts):
    code, out = ctx.executor.capture(['docker', 'inspect', '-f', MOUNT_FORMAT, name])
    if code:
        return
    image, _, mount_text = out.strip().partition('|')
    current = {item for item in mount_text.split(',') if item}
    wanted = {f'{host_path}:{container_path}' for host_path, container_path in mounts}
    explicit = ctx.settings.get('image')
    if current != wanted or (explicit and explicit != image):
        term.warn(
            f"container '{name}' uses image {image} with mounts {sorted(current) or 'none'}; "
            f"requested {explicit or 'auto-selected image'} with {sorted(wanted) or 'none'}. "
            'Use --recreate to apply the requested setup.')


def cmd_run(ctx):
    """Start the SUAVE container or reuse an existing one."""
    require_docker(ctx)
    settings, executor = ctx.settings, ctx.executor
    name, mode = settings.get('container_name'), settings.get('run_mode')
    mounts = build_mounts(ctx)
    state = container_state(executor, name)
    if state and getattr(ctx.args, 'recreate', False):
        code = executor.run(['docker', 'rm', '-f', name])
        if code:
            return code
        state = None
    if state is not None:
        _warn_if_different(ctx, name, mounts)
        if state == 'running':
            term.info(f"container '{name}' is already running; reusing it")
            return 0
        term.info(f"starting existing container '{name}'")
        return executor.run(['docker', 'start', name])
    image = ensure_image(ctx)
    if settings.get_bool('mount_results') and not executor.dry_run:
        Path(settings.get('host_results_dir')).expanduser().mkdir(parents=True, exist_ok=True)
    if any(path == settings.get('container_src_dir') for _, path in mounts):
        term.info("The source is mounted, but the image's build/ and install/ come from its "
                  "baked-in copy: run 'suave build' after changing C++, messages or "
                  'entry points.')
    code = executor.run(run_argv(name, image, mode, mounts, ctx.passthrough))
    if code == 0 and mode == 'detached':
        term.info(f"container '{name}' started; open a shell with: suave docker shell")
    return code


def _require_running(ctx):
    name = ctx.settings.get('container_name')
    if container_state(ctx.executor, name) != 'running':
        raise CliError(f"container '{name}' is not running; start it with: suave docker run")
    return name


def cmd_shell(ctx):
    """Open an interactive, ROS-sourced shell in the running container."""
    require_docker(ctx)
    name = _require_running(ctx)
    target = Target('docker', ctx.settings.get('container_workspace'), CONTAINER_ROS_SETUP, name)
    return ctx.executor.run(target.argv('exec bash', tty=True))


def cmd_stop(ctx):
    """Stop the container and optionally remove it."""
    require_docker(ctx)
    name = ctx.settings.get('container_name')
    state = container_state(ctx.executor, name)
    if state is None:
        raise CliError(f"container '{name}' does not exist")
    if state == 'running':
        code = ctx.executor.run(['docker', 'stop', name])
        if code:
            return code
    if getattr(ctx.args, 'rm', False):
        return ctx.executor.run(['docker', 'rm', name])
    return 0


def cmd_status(ctx):
    """Print whether the container exists, runs, and what it mounts."""
    require_docker(ctx)
    name = ctx.settings.get('container_name')
    state = container_state(ctx.executor, name)
    if state is None:
        print(f"container '{name}' does not exist")
        return 1
    _, out = ctx.executor.capture(['docker', 'inspect', '-f', MOUNT_FORMAT, name])
    image, _, mount_text = out.strip().partition('|')
    print(f"container '{name}': {state}")
    print(f'image: {image}')
    for item in (m for m in mount_text.split(',') if m):
        print(f'mount: {item}')
    return 0


def add_parsers(sub, common):
    """Add suave docker run, shell, stop and status."""
    raw = argparse.RawDescriptionHelpFormatter
    run = sub.add_parser('run', parents=[common], help="start or reuse the 'suave' container",
                         description=RUN_DESCRIPTION, epilog=RUN_EPILOG, formatter_class=raw)
    run.add_argument('--image', default=argparse.SUPPRESS,
                     help='image to run (default: auto-select, see above)')
    mode = run.add_mutually_exclusive_group()
    mode.add_argument('--detach', dest='run_mode', action='store_const', const='detached',
                      default=argparse.SUPPRESS, help='run in the background (default)')
    mode.add_argument('--interactive', dest='run_mode', action='store_const',
                      const='interactive', default=argparse.SUPPRESS,
                      help='open a shell and remove the container on exit')
    src = run.add_mutually_exclusive_group()
    src.add_argument('--mount-src', dest='mount_src', action='store_true',
                     default=argparse.SUPPRESS, help='mount the checkout (default)')
    src.add_argument('--no-mount-src', dest='mount_src', action='store_false',
                     default=argparse.SUPPRESS, help='use the code baked into the image')
    run.add_argument('--src', metavar='PATH',
                     help='checkout to mount instead of SUAVE_ROOT (implies --mount-src)')
    results = run.add_mutually_exclusive_group()
    results.add_argument('--mount-results', dest='mount_results', action='store_true',
                         default=argparse.SUPPRESS, help='mount the results folder (default)')
    results.add_argument('--no-mount-results', dest='mount_results', action='store_false',
                         default=argparse.SUPPRESS, help='do not mount the results folder')
    run.add_argument('--results-dir', metavar='PATH', default=argparse.SUPPRESS,
                     help='host results folder to mount (default: ~/suave/results)')
    run.add_argument('--recreate', action='store_true',
                     help='remove an existing container with the same name first')
    run.set_defaults(func=cmd_run, accepts_passthrough=True)

    shell = sub.add_parser('shell', parents=[common],
                           help='open a ROS-sourced shell in the running container')
    shell.set_defaults(func=cmd_shell)
    stop = sub.add_parser('stop', parents=[common], help='stop the container')
    stop.add_argument('--rm', action='store_true', help='also remove the container')
    stop.set_defaults(func=cmd_stop)
    status = sub.add_parser('status', parents=[common],
                            help='show whether the container exists, runs and what it mounts')
    status.set_defaults(func=cmd_status)
