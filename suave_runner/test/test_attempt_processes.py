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


"""Stop recorded launch sessions, escalating to SIGKILL when ignored."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from suave_runner.attempt_processes import confirm_stopped
from suave_runner.attempt_processes import live_gz_sim_processes
from suave_runner.attempt_processes import process_identity
from suave_runner.attempt_processes import stop_sessions

IGNORE_SIGTERM = (
    'import signal, sys, time\n'
    'signal.signal(signal.SIGTERM, signal.SIG_IGN)\n'
    'print("ready", flush=True)\n'
    'time.sleep(60)\n')


def _spawn(log_dir, code):
    # Never call register_session here: its setsid() would move pytest.
    child = subprocess.Popen(
        [sys.executable, '-c', code], start_new_session=True,
        stdout=subprocess.PIPE, text=True)
    _, session, start = process_identity(child.pid)
    directory = Path(log_dir) / 'simulation'
    directory.mkdir(parents=True)
    (directory / 'session.json').write_text(json.dumps({
        'session': session,
        'start': start,
        'boot': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        'namespace': os.readlink('/proc/self/ns/pid'),
    }))
    return child


def _stop(child):
    if child.poll() is None:
        child.kill()
    child.wait()
    child.stdout.close()


def test_stop_sessions_terminates_with_sigterm(tmp_path):
    child = _spawn(tmp_path, 'print("ready", flush=True)\n'
                             'import time\ntime.sleep(60)\n')
    try:
        assert child.stdout.readline().strip() == 'ready'
        stop_sessions(tmp_path, timeout=5.0, poll=0.05)
        assert child.wait(timeout=5) == -signal.SIGTERM
    finally:
        _stop(child)


def test_stop_sessions_escalates_to_sigkill(tmp_path):
    child = _spawn(tmp_path, IGNORE_SIGTERM)
    try:
        assert child.stdout.readline().strip() == 'ready'
        stop_sessions(tmp_path, timeout=0.5, poll=0.05)
        assert child.wait(timeout=5) == -signal.SIGKILL
    finally:
        _stop(child)


def test_unreaped_zombies_do_not_count_as_live(tmp_path):
    child = _spawn(tmp_path, 'pass')
    try:
        deadline = time.monotonic() + 5
        while process_identity(child.pid)[0] != 'Z':
            assert time.monotonic() < deadline, 'child never exited'
            time.sleep(0.05)
        confirm_stopped(tmp_path)
        started = time.monotonic()
        stop_sessions(tmp_path, timeout=5.0, poll=0.05)
        assert time.monotonic() - started < 1.0
    finally:
        _stop(child)


def test_live_gz_sim_processes_ignores_zombies_and_other_commands():
    gz = subprocess.Popen(
        ['bash', '-c', 'exec -a "gz sim -r -s min_pipes.world" sleep 60'])
    other = subprocess.Popen(['bash', '-c', 'exec -a "gz_simulator" sleep 60'])
    try:
        deadline = time.monotonic() + 5
        while gz.pid not in live_gz_sim_processes():
            assert time.monotonic() < deadline, 'gz sim never listed'
            time.sleep(0.05)
        assert other.pid not in live_gz_sim_processes()
        gz.kill()
        while process_identity(gz.pid)[0] != 'Z':
            assert time.monotonic() < deadline, 'gz sim never exited'
            time.sleep(0.05)
        assert gz.pid not in live_gz_sim_processes()
    finally:
        for child in (gz, other):
            if child.poll() is None:
                child.kill()
            child.wait()
