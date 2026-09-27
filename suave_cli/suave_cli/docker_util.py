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

"""Small Docker queries shared by several commands."""

from suave_cli.errors import CliError

RUNNING_FORMAT = '{{.State.Running}}'


def docker_available(executor):
    """Return True when the docker CLI works and the daemon answers."""
    code, _ = executor.capture(['docker', 'info', '--format', '{{.ServerVersion}}'])
    return code == 0


def container_state(executor, name):
    """Return 'running', 'stopped', or None when the container does not exist."""
    code, out = executor.capture(['docker', 'inspect', '-f', RUNNING_FORMAT, name])
    if code != 0:
        return None
    return 'running' if out.strip() == 'true' else 'stopped'


def require_host(ctx):
    """Refuse to run docker commands inside the SUAVE container."""
    if ctx.in_container:
        raise CliError('docker commands must be run on the host, not inside the SUAVE container')


def require_docker(ctx):
    """Require the host context and a reachable Docker daemon."""
    require_host(ctx)
    if not docker_available(ctx.executor):
        raise CliError('Docker is not available: install Docker or start the Docker daemon')
