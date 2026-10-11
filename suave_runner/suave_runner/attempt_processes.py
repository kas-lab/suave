# Copyright 2026 KAS-lab
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

"""Track isolated launch sessions before they can spawn recording writers."""

import json
import os
from pathlib import Path

from suave_runner.run_status import write_status


def process_identity(pid):
    """Read Linux process state, session and birth time."""
    text = Path(f'/proc/{pid}/stat').read_text()
    fields = text[text.rindex(')') + 2:].split()
    return fields[0], int(fields[3]), int(fields[19])


def register_session(log_dir):
    """Persist the child's session identity before launch actions spawn."""
    os.setsid()
    directory = Path(log_dir)
    directory.mkdir(parents=True, exist_ok=True)
    _, session, start = process_identity(os.getpid())
    write_status(directory / 'session.json', {
        'session': session,
        'start': start,
        'boot': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        'namespace': os.readlink('/proc/self/ns/pid'),
    })


def confirm_stopped(log_dir):
    """Refuse recovery while a recorded launch session has live members."""
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    namespace = os.readlink('/proc/self/ns/pid')
    for path in Path(log_dir).glob('*/session.json'):
        record = json.loads(path.read_text())
        if record['boot'] != boot:
            continue
        if record['namespace'] != namespace:
            raise RuntimeError(
                'Cannot verify writers from another PID namespace: '
                f'{path}')
        _confirm_session_stopped(record)


def _confirm_session_stopped(record):
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            state, session, start = process_identity(proc.name)
        except FileNotFoundError:
            continue
        live_member = (
            state != 'Z' and session == record['session'] and
            start >= record['start'])
        if live_member:
            raise RuntimeError(
                f'Attempt writer/session still alive: PID {proc.name}')
