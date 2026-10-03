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

"""Host display (X11) and GPU options for suave docker run."""

from dataclasses import dataclass, field
import os
from pathlib import Path
import tempfile

from suave_cli import term
from suave_cli.errors import CliError

X11_SOCKET = '/tmp/.X11-unix'
LOCALTIME = '/etc/localtime'
DRI = '/dev/dri'
CONTAINER_XAUTH_DIR = '/tmp/.suave-xauth'
XAUTH_FILE = 'Xauthority'
CONTAINER_UID = 1000
NVIDIA_ARGS = ('--gpus', 'all', '--runtime=nvidia',
               '-e', 'NVIDIA_VISIBLE_DEVICES=all', '-e', 'NVIDIA_DRIVER_CAPABILITIES=all')
RUNTIMES_FORMAT = '{{range $name, $_ := .Runtimes}}{{$name}} {{end}}'


@dataclass
class RunOptions:
    """docker run arguments and the bind mounts they add, as (host, container) pairs."""

    args: list = field(default_factory=list)
    mounts: list = field(default_factory=list)
    display: str = ''

    def mount(self, host, dest, read_only=False):
        """Add one bind mount."""
        self.args += ['-v', f'{host}:{dest}' + (':ro' if read_only else '')]
        self.mounts.append((host, dest))


def xauth_dir(name, env=None):
    """Return the host folder holding the X cookie for container name."""
    env = os.environ if env is None else env
    cache = Path(env.get('XDG_CACHE_HOME') or Path.home() / '.cache').expanduser()
    return cache / 'suave' / 'xauth' / name


def wild_cookies(nlist):
    """Return Xauthority records from xauth nlist output, valid for any hostname."""
    records = b''
    for line in nlist.splitlines():
        fields = line.split()
        if not fields:
            continue
        try:
            records += bytes.fromhex('ffff' + ''.join(fields[1:]))
        except ValueError:
            continue
    return records


def write_private(path, data):
    """Write data to path atomically, readable only by the current user."""
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    os.chmod(path.parent, 0o700)
    handle, tmp_name = tempfile.mkstemp(dir=path.parent, prefix='.tmp-')
    try:
        with os.fdopen(handle, 'wb') as stream:
            stream.write(data)
        os.replace(tmp_name, path)
    except OSError:
        if os.path.exists(tmp_name):
            os.unlink(tmp_name)
        raise


def refresh_cookie(ctx, name, display):
    """Store the display's X cookie for container name; return the host folder."""
    folder = xauth_dir(name)
    code, out = ctx.executor.capture(['xauth', 'nlist', display])
    if code == 127:
        term.warn('xauth is not installed, so the container gets no X cookie; '
                  "install the 'xauth' package if GUI apps cannot open the display")
    cookies = wild_cookies(out) if code == 0 else b''
    if code != 127 and not cookies:
        term.info(f'no X cookie found for DISPLAY={display}; GUI apps work only if the '
                  f'X server already allows uid {CONTAINER_UID}')
    if ctx.executor.dry_run:
        return folder
    try:
        write_private(folder / XAUTH_FILE, cookies)
    except OSError as exc:
        raise CliError(f'cannot write the X cookie to {folder} ({exc})') from None
    return folder


def check_nvidia_runtime(ctx):
    """Raise CliError when Docker has no nvidia runtime."""
    code, out = ctx.executor.capture(['docker', 'info', '-f', RUNTIMES_FORMAT])
    if code == 0 and 'nvidia' not in out.split():
        raise CliError('Docker has no nvidia runtime: install the NVIDIA Container Toolkit, '
                       'or run without the GPU with --gpu none '
                       '(to keep it: suave config set gpu none)')


def run_options(ctx, name, env=None):
    """Return the display and GPU options for container name, refreshing its X cookie."""
    env = os.environ if env is None else env
    options = RunOptions()
    if ctx.settings.get('gpu') == 'nvidia':
        options.args += list(NVIDIA_ARGS)
        options.mount(DRI, DRI)
    display = env.get('DISPLAY', '')
    if not display:
        term.info('DISPLAY is not set: the container gets no access to a host display')
    else:
        if display.partition(':')[0]:
            term.warn(f'DISPLAY={display} is a network display; the container can only reach '
                      f'local displays through {X11_SOCKET}')
        if hasattr(os, 'getuid') and os.getuid() != CONTAINER_UID:
            term.warn(f'your uid is {os.getuid()}, but the container user has uid '
                      f'{CONTAINER_UID} and cannot read the X cookie')
        folder = refresh_cookie(ctx, name, display)
        options.display = display
        options.args += ['-e', f'DISPLAY={display}', '-e', 'QT_X11_NO_MITSHM=1',
                         '-e', f'XAUTHORITY={CONTAINER_XAUTH_DIR}/{XAUTH_FILE}']
        options.mount(X11_SOCKET, X11_SOCKET)
        options.mount(str(folder), CONTAINER_XAUTH_DIR, read_only=True)
    options.mount(LOCALTIME, LOCALTIME, read_only=True)
    return options
