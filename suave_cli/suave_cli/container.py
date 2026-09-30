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
from pathlib import Path, PurePosixPath

from suave_cli import config, term
from suave_cli.docker_util import container_state, docker_available, require_docker, require_host
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
~/suave/results at the container's results folder, plus every mount
saved with 'suave docker mount add'.
"""

RUN_EPILOG = """\
examples:
  suave docker run                              detached, default mounts
  suave docker run --interactive --no-mount-src throwaway shell on the baked-in code
  suave docker run --image suave-headless:exp1 --recreate
  suave docker run --mount ../my_package         one-off extra mount into the workspace src/
  suave docker run -- --network host            extra docker run arguments
"""


MOUNT_EPILOG = """\
examples:
  suave docker mount add ../my_package          mount at <container_workspace>/src/my_package
  suave docker mount add ~/data --to /data
  suave docker mount remove ../my_package       by host path or container path
  suave docker mount list

Mounts only apply when the container is created: run 'suave docker run --recreate'.
"""


def default_mounts(ctx):
    """Return the checkout and results bind mounts for docker run."""
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


def parse_mount(ctx, host, dest=None):
    """Return a (host, container) mount for host; dest defaults to the workspace src/."""
    host_path = Path(host).expanduser().resolve()
    if not dest:
        workspace = PurePosixPath(ctx.settings.get('container_workspace'))
        dest = str(workspace / 'src' / host_path.name)
    entry = config.normalize('extra_mounts', f'{host_path}:{dest}', 'command line')
    return tuple(entry.split(':', 1))


def saved_mounts(entries):
    """Return (host, container) pairs for normalized extra_mounts entries."""
    return [tuple(entry.split(':', 1)) for entry in entries]


def extra_mounts(ctx):
    """Return the saved extra mounts and the --mount ones for docker run."""
    args = ctx.args
    mounts = []
    if not getattr(args, 'no_extra_mounts', False):
        mounts += saved_mounts(ctx.settings.get_list('extra_mounts'))
    for spec in getattr(args, 'extra_mount', None) or []:
        host, _, dest = spec.partition(':')
        mounts.append(parse_mount(ctx, host, dest))
    return mounts


def build_mounts(ctx):
    """Return the (host path, container path) bind mounts for docker run."""
    return default_mounts(ctx) + extra_mounts(ctx)


def check_mount(host, dest, taken):
    """Refuse a missing host path or a destination that overlaps a path in taken."""
    if not Path(host).exists():
        raise CliError(f'mount source {host} does not exist')
    for other in taken:
        if PurePosixPath(dest).is_relative_to(other) or PurePosixPath(other).is_relative_to(dest):
            raise CliError(f'mount destination {dest} overlaps {other}; choose another with --to')


def check_extra_mounts(defaults, extras):
    """Validate every extra mount against the default mounts and the ones before it."""
    taken = [dest for _, dest in defaults]
    for host, dest in extras:
        check_mount(host, dest, taken)
        taken.append(dest)


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
    defaults, extras = default_mounts(ctx), extra_mounts(ctx)
    check_extra_mounts(defaults, extras)
    mounts = defaults + extras
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
    workspace_src = PurePosixPath(settings.get('container_workspace')) / 'src'
    for _, dest in extras:
        if PurePosixPath(dest).parent == workspace_src:
            term.info(f"{dest} is in the workspace: build it with 'suave build <package>'.")
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


def _recreate_hint(ctx):
    if not docker_available(ctx.executor):
        return
    name = ctx.settings.get('container_name')
    if container_state(ctx.executor, name) is not None:
        term.info(f"container '{name}' already exists; apply the change with: "
                  'suave docker run --recreate')


def _save_mounts(ctx, entries):
    values = config.load_file(ctx.config_path)
    if entries:
        values['extra_mounts'] = '\n'.join(entries)
    else:
        values.pop('extra_mounts', None)
    return config.save_file(ctx.config_path, values)


def cmd_mount_add(ctx):
    """Save an extra bind mount for suave docker run."""
    require_host(ctx)
    host, dest = parse_mount(ctx, ctx.args.path, ctx.args.to)
    entries = ctx.settings.get_list('extra_mounts')
    entry = f'{host}:{dest}'
    if entry in entries:
        term.info(f'{host} is already mounted at {dest}')
        return 0
    taken = [path for _, path in default_mounts(ctx) + saved_mounts(entries)]
    check_mount(host, dest, taken)
    if not _save_mounts(ctx, entries + [entry]):
        return 1
    print(f'mount: {host} -> {dest}  ({ctx.config_path})')
    _recreate_hint(ctx)
    return 0


def cmd_mount_remove(ctx):
    """Remove a saved extra bind mount, matched by host or container path."""
    require_host(ctx)
    raw = ctx.args.path
    host = str(Path(raw).expanduser().resolve())
    entries = config.split_list(config.load_file(ctx.config_path).get('extra_mounts', ''))
    kept = [entry for entry in entries
            if raw != entry and entry.partition(':')[0] != host and entry.partition(':')[2] != raw]
    if len(kept) == len(entries):
        saved = ', '.join(entries) or 'none'
        raise CliError(f'no saved mount matches {raw!r}; saved mounts: {saved}')
    if not _save_mounts(ctx, kept):
        return 1
    for entry in entries:
        if entry not in kept:
            print(f'removed mount: {entry}')
    if ctx.settings is not None:
        _recreate_hint(ctx)
    return 0


def cmd_mount_list(ctx):
    """Print the default and saved extra bind mounts."""
    require_host(ctx)
    rows = [(host, dest, 'default') for host, dest in default_mounts(ctx)]
    rows += [(host, dest, 'extra_mounts' + ('' if Path(host).exists() else ', missing'))
             for host, dest in saved_mounts(ctx.settings.get_list('extra_mounts'))]
    if not rows:
        print('no mounts')
    for host, dest, label in rows:
        print(f'{host} -> {dest}  ({label})')
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
    run.add_argument('--mount', dest='extra_mount', action='append', metavar='HOST[:CONTAINER]',
                     help='extra bind mount for this run, added to the saved ones '
                          '(default CONTAINER: <container_workspace>/src/<name>)')
    run.add_argument('--no-extra-mounts', action='store_true',
                     help="skip the mounts saved with 'suave docker mount add'")
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

    mount = sub.add_parser('mount', parents=[common], help='manage extra bind mounts for run',
                           description='Save extra bind mounts in the extra_mounts setting. '
                                       'Host only.',
                           epilog=MOUNT_EPILOG, formatter_class=raw)
    mount.set_defaults(skip_first_run=True)
    mount_sub = mount.add_subparsers(dest='mount_command', metavar='ACTION', required=True)
    add = mount_sub.add_parser('add', parents=[common], help='save an extra mount')
    add.add_argument('path', help='host file or folder to mount')
    add.add_argument('--to', metavar='CONTAINER',
                     help='absolute container path (default: <container_workspace>/src/<name>)')
    add.set_defaults(func=cmd_mount_add)
    remove = mount_sub.add_parser('remove', parents=[common], help='remove a saved extra mount')
    remove.add_argument('path', help='host path or container path of the mount')
    remove.set_defaults(func=cmd_mount_remove, tolerate_bad_config=True)
    show = mount_sub.add_parser('list', parents=[common], help='list default and extra mounts')
    show.set_defaults(func=cmd_mount_list)
