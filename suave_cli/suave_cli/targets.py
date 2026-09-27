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

"""Choose where ROS commands run and build the matching command lines."""

from dataclasses import dataclass
from pathlib import Path
import shlex

from suave_cli import term
from suave_cli.context import find_workspace
from suave_cli.docker_util import container_state, docker_available
from suave_cli.errors import CliError

CONTAINER_ROS_SETUP = '/opt/ros/humble/setup.bash'


@dataclass(frozen=True)
class Target:
    """A place where ROS commands run: 'local' (this machine) or 'docker'."""

    kind: str
    workspace: str
    ros_setup: str
    container: str = ''

    def script(self, body):
        """Wrap body so it runs with ROS and the workspace overlay sourced."""
        return (
            f'source {shlex.quote(self.ros_setup)} && cd {shlex.quote(self.workspace)} && '
            'if [ -f install/setup.bash ]; then source install/setup.bash; fi && '
            f'{{ {body}; }}')

    def argv(self, body, tty=False):
        """Return the full command line that runs body in this target."""
        if self.kind == 'docker':
            flags = ['-it'] if tty else []
            return ['docker', 'exec', *flags, self.container, 'bash', '-lc', self.script(body)]
        return ['bash', '-lc', self.script(body)]

    def describe(self):
        """Return a short human-readable description of the target."""
        if self.kind == 'docker':
            return f"container '{self.container}' ({self.workspace})"
        return f'local workspace {self.workspace}'


def _local_target(ctx):
    settings = ctx.settings
    ros_setup = Path(settings.get('ros_setup')).expanduser()
    if not ros_setup.is_file():
        raise CliError(f'ROS setup file {ros_setup} was not found')
    workspace = find_workspace(ctx.suave_root, settings.get('host_workspace'))
    if workspace is None:
        raise CliError(
            f'no colcon workspace contains {ctx.suave_root} in its src/ folder; '
            'set host_workspace or pass --workspace')
    return Target('local', str(workspace), str(ros_setup))


def _docker_target(ctx):
    settings = ctx.settings
    return Target('docker', settings.get('container_workspace'), CONTAINER_ROS_SETUP,
                  settings.get('container_name'))


def select_target(ctx):
    """Return where ROS commands run, following the exec setting."""
    if ctx.in_container:
        return _local_target(ctx)
    mode = ctx.settings.get('exec')
    if mode == 'host':
        return _local_target(ctx)
    name = ctx.settings.get('container_name')
    docker_ok = docker_available(ctx.executor)
    running = docker_ok and container_state(ctx.executor, name) == 'running'
    if mode == 'container':
        if not docker_ok:
            raise CliError('exec is container but Docker is not available')
        if not running:
            raise CliError(f"container '{name}' is not running; start it with: suave docker run")
        return _docker_target(ctx)
    if running:
        target = _docker_target(ctx)
    else:
        try:
            target = _local_target(ctx)
        except CliError as exc:
            raise CliError(
                f"nowhere to run ROS commands: container '{name}' is not running and {exc}. "
                "Start the container with 'suave docker run', or configure local runs "
                "with 'suave config init'.") from None
    term.info(f'Running in {target.describe()}')
    return target


def target_results_dir(ctx, target):
    """Return the results folder as seen from inside target."""
    if target.kind == 'docker':
        return ctx.settings.get('container_results_dir')
    return str(Path(ctx.settings.get('host_results_dir')).expanduser())


def run_in_target(ctx, body, target=None):
    """Run body in target (selected when not given) and return its exit code."""
    target = target or select_target(ctx)
    return ctx.executor.run(target.argv(body, tty=term.stdin_is_tty()))
