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

"""Load, resolve and store suave CLI settings."""

import configparser
from dataclasses import dataclass
import os
from pathlib import Path
import tempfile

from suave_cli import term
from suave_cli.errors import CliError

SECTION = 'suave'
TRUE_WORDS = ('1', 'true', 'yes', 'on')
FALSE_WORDS = ('0', 'false', 'no', 'off')


@dataclass(frozen=True)
class KeySpec:
    """One configuration key: default, description and validation rules."""

    default: str
    description: str
    env: str = ''
    choices: tuple = ()
    kind: str = 'str'


KEYS = {
    'exec': KeySpec('auto', 'where ROS commands run: auto, container or host',
                    env='SUAVE_EXEC', choices=('auto', 'container', 'host')),
    'container_name': KeySpec('suave', 'name of the SUAVE container',
                              env='SUAVE_CONTAINER_NAME'),
    'image': KeySpec('', 'image for suave docker run (empty = auto-select)',
                     env='SUAVE_IMAGE'),
    'run_mode': KeySpec('detached', 'suave docker run mode: detached or interactive',
                        choices=('detached', 'interactive')),
    'mount_src': KeySpec('true', 'mount this checkout into the container', kind='bool'),
    'mount_results': KeySpec('true', 'mount the results folder into the container',
                             kind='bool'),
    'host_results_dir': KeySpec('~/suave/results', 'results folder on this machine'),
    'host_workspace': KeySpec('', 'colcon workspace for local runs (empty = auto-detect)',
                              env='SUAVE_WORKSPACE'),
    'ros_setup': KeySpec('/opt/ros/humble/setup.bash', 'ROS setup script for local runs'),
    'container_results_dir': KeySpec('/home/ubuntu-user/suave/results',
                                     'results folder inside the container'),
    'container_src_dir': KeySpec('/home/ubuntu-user/suave_ws/src/suave',
                                 'checkout location inside the container'),
    'container_workspace': KeySpec('/home/ubuntu-user/suave_ws',
                                   'colcon workspace inside the container'),
}

FLAG_TO_KEY = {
    'exec_mode': 'exec',
    'container': 'container_name',
    'image': 'image',
    'run_mode': 'run_mode',
    'mount_src': 'mount_src',
    'mount_results': 'mount_results',
    'results_dir': 'host_results_dir',
    'workspace': 'host_workspace',
}

HOST_WIZARD_KEYS = ('exec', 'container_name', 'image', 'run_mode', 'mount_src',
                    'mount_results', 'host_results_dir', 'host_workspace')
CONTAINER_WIZARD_KEYS = ('exec', 'host_workspace')


def config_path(suave_root, container):
    """Return the config file used on the host or inside the container."""
    name = 'container.ini' if container else 'host.ini'
    return Path(suave_root) / '.config' / name


def normalize(key, value, source):
    """Validate a raw value for key and return its canonical string form."""
    spec = KEYS.get(key)
    if spec is None:
        raise CliError(f'unknown config key {key!r} (from {source})')
    value = str(value).strip()
    if spec.kind == 'bool':
        lowered = value.lower()
        if lowered in TRUE_WORDS:
            return 'true'
        if lowered in FALSE_WORDS:
            return 'false'
        raise CliError(f'{key}: expected true or false, got {value!r} (from {source})')
    if spec.choices and value not in spec.choices:
        raise CliError(
            f'{key}: expected one of {", ".join(spec.choices)}, got {value!r} (from {source})')
    return value


def load_file(path):
    """Return the [suave] section of path as a dict, or {} when it is missing."""
    parser = configparser.ConfigParser(interpolation=None)
    try:
        parser.read(path, encoding='utf-8')
    except configparser.Error as exc:
        raise CliError(f'cannot parse {path}: {exc}') from None
    if not parser.has_section(SECTION):
        return {}
    return dict(parser.items(SECTION))


def _header():
    lines = ['# suave CLI configuration. Keys that are not set use these defaults:']
    lines += [f'#   {key} = {spec.default}' for key, spec in KEYS.items()]
    return '\n'.join(lines) + '\n\n'


def save_file(path, values):
    """Write values atomically; warn and return False when that is impossible."""
    path = Path(path)
    parser = configparser.ConfigParser(interpolation=None)
    parser[SECTION] = values
    tmp_name = None
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        handle, tmp_name = tempfile.mkstemp(dir=path.parent, prefix='.tmp-', suffix='.ini')
        with os.fdopen(handle, 'w', encoding='utf-8') as stream:
            stream.write(_header())
            parser.write(stream)
        os.replace(tmp_name, path)
    except OSError as exc:
        if tmp_name and os.path.exists(tmp_name):
            os.unlink(tmp_name)
        term.warn(f'cannot write {path} ({exc}); continuing with the current settings')
        return False
    return True


class Settings:
    """Resolved settings plus the source each value came from."""

    def __init__(self, values, sources):
        self._values = values
        self._sources = sources

    def get(self, key):
        """Return the value of key as a string."""
        return self._values[key]

    def get_bool(self, key):
        """Return the value of a boolean key."""
        return self._values[key] == 'true'

    def source(self, key):
        """Return where the value of key came from."""
        return self._sources[key]

    def items(self):
        """Return (key, value, source) for every key, in KEYS order."""
        return [(key, self._values[key], self._sources[key]) for key in KEYS]


def resolve(flags, env, file_values, file_label='file'):
    """Resolve every key: flag, then env var, then config file, then default."""
    for key in file_values:
        if key not in KEYS:
            term.warn(f'unknown config key {key!r} in {file_label}; ignoring it')
    values, sources = {}, {}
    for key, spec in KEYS.items():
        if flags.get(key) is not None:
            raw, source = flags[key], 'flag'
        elif spec.env and env.get(spec.env):
            raw, source = env[spec.env], f'env {spec.env}'
        elif key in file_values:
            raw, source = file_values[key], file_label
        else:
            raw, source = spec.default, 'default'
        values[key] = normalize(key, raw, source)
        sources[key] = source
    return Settings(values, sources)


def run_wizard(path, container, input_fn=input):
    """Ask for the configurable defaults and store the answers in path."""
    current = load_file(path)
    values = dict(current)
    for key in CONTAINER_WIZARD_KEYS if container else HOST_WIZARD_KEYS:
        spec = KEYS[key]
        default = current.get(key, spec.default)
        while True:
            answer = term.ask_value(f'{key} ({spec.description})', default, input_fn=input_fn)
            try:
                value = normalize(key, answer, 'setup')
                break
            except CliError as exc:
                term.error(str(exc))
        if value == spec.default:
            values.pop(key, None)
        else:
            values[key] = value
    save_file(path, values)
    return values


def maybe_first_run(path, container, interactive, input_fn=input):
    """Offer the setup wizard when no config file exists yet."""
    if Path(path).exists():
        return
    if not interactive:
        term.info(f"tip: no suave config at {path}; run 'suave config init' to create one")
        return
    if term.ask_yes_no('No suave config found. Configure it now?', input_fn=input_fn):
        run_wizard(path, container, input_fn=input_fn)
    else:
        save_file(path, {})
        term.info("Run 'suave config init' any time to change defaults.")
