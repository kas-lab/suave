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

"""Command-line entry point for the suave CLI."""

import argparse
import os
import sys

from suave_cli import analyze, config, docker_cmd, ros_build, runner, selftest, term
from suave_cli.context import AppContext, in_container, resolve_suave_root
from suave_cli.errors import CliError
from suave_cli.executor import Executor

COMMAND_MODULES = [docker_cmd, ros_build, runner, analyze, config, selftest]

DESCRIPTION = """\
SUAVE command-line helper.

Builds and runs the SUAVE Docker images, builds and tests the SUAVE packages,
and starts, resumes and lists experiment campaigns. ROS commands run in the
SUAVE container or in a colcon workspace on this machine (see --exec).
"""

EPILOG = """\
examples:
  suave docker build              build suave-headless:latest
  suave docker run                start the 'suave' container in the background
  suave test suave_runner         build and test one package
  suave batch resume --latest     resume the most recent batch
  suave config show               show settings and where they come from

Arguments after '--' are passed to the wrapped tool (docker, colcon, ros2).
Run 'suave COMMAND --help' for details on a command.
"""


def common_parser():
    """Return the parser holding the global options shared by every command."""
    common = argparse.ArgumentParser(add_help=False)
    group = common.add_argument_group('global options')
    group.add_argument('--exec', dest='exec_mode', choices=('auto', 'container', 'host'),
                       default=argparse.SUPPRESS,
                       help='where ROS commands run (default: auto)')
    group.add_argument('--container', metavar='NAME', default=argparse.SUPPRESS,
                       help='SUAVE container name (default: suave)')
    group.add_argument('--workspace', metavar='PATH', default=argparse.SUPPRESS,
                       help='colcon workspace for local runs (default: auto-detect)')
    group.add_argument('--suave-root', metavar='PATH', default=argparse.SUPPRESS,
                       help='SUAVE checkout to use (default: $SUAVE_ROOT)')
    group.add_argument('--dry-run', action='store_true', default=argparse.SUPPRESS,
                       help='print commands instead of running them')
    group.add_argument('-y', '--yes', action='store_true', default=argparse.SUPPRESS,
                       help='answer yes to prompts, e.g. pulling an image')
    group.add_argument('-v', '--verbose', action='store_true', default=argparse.SUPPRESS,
                       help='print every command before running it')
    return common


def build_parser():
    """Return the complete argument parser."""
    common = common_parser()
    parser = argparse.ArgumentParser(
        prog='suave', description=DESCRIPTION, epilog=EPILOG, parents=[common],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    subparsers = parser.add_subparsers(title='commands', metavar='COMMAND')
    for module in COMMAND_MODULES:
        module.register(subparsers, common)
    return parser


def split_passthrough(argv):
    """Split argv at the first '--' into (own arguments, passthrough arguments)."""
    if '--' in argv:
        index = argv.index('--')
        return argv[:index], argv[index + 1:]
    return argv, []


def make_context(args, env, passthrough, input_fn=input, runner=None):
    """Build the AppContext for one invocation."""
    root = resolve_suave_root(getattr(args, 'suave_root', None), env)
    container = in_container(env)
    interactive = term.is_interactive()
    path = config.config_path(root, container)
    if not getattr(args, 'skip_first_run', False):
        config.maybe_first_run(path, container, interactive, input_fn=input_fn)
    flags = {key: getattr(args, dest)
             for dest, key in config.FLAG_TO_KEY.items() if hasattr(args, dest)}
    try:
        settings = config.resolve(flags, env, config.load_file(path),
                                  file_label=f'file {path.name}')
    except CliError:
        if not getattr(args, 'tolerate_bad_config', False):
            raise
        settings = None
    executor = Executor(dry_run=getattr(args, 'dry_run', False),
                        verbose=getattr(args, 'verbose', False), runner=runner)
    return AppContext(suave_root=root, in_container=container, interactive=interactive,
                      settings=settings, executor=executor, args=args,
                      passthrough=list(passthrough), config_path=path, input_fn=input_fn)


def main(argv=None, env=None, input_fn=input, runner=None):
    """Run the CLI and return its exit code."""
    argv = list(sys.argv[1:] if argv is None else argv)
    env = os.environ if env is None else env
    own_args, passthrough = split_passthrough(argv)
    parser = build_parser()
    args = parser.parse_args(own_args)
    if not hasattr(args, 'func'):
        parser.print_help()
        return 0
    if passthrough and not getattr(args, 'accepts_passthrough', False):
        parser.error("this command does not accept extra arguments after '--'")
    try:
        ctx = make_context(args, env, passthrough, input_fn=input_fn, runner=runner)
        return args.func(ctx)
    except CliError as exc:
        term.error(str(exc))
        return exc.code
    except KeyboardInterrupt:
        term.error('interrupted')
        return 130
