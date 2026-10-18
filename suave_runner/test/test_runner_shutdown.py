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

"""Regression checks for cooperative signal handling and exhaustive cleanup."""

import json
import os
import signal
import subprocess
import sys
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from suave_runner.attempt_processes import confirm_stopped
from suave_runner.suave_runner import ExperimentRunnerNode


def test_shutdown_signals_all_launchers_before_waiting():
    """Request every stop even when the first launcher remains alive."""
    calls = []
    processes = []
    for pid in (1, 2):
        process = SimpleNamespace(
            pid=pid, is_alive=lambda: True,
            join=lambda timeout, pid=pid: calls.append(('join', pid)))
        event = SimpleNamespace(
            set=lambda pid=pid: calls.append(('stop', pid)))
        processes.append((process, event, str(pid)))
    runner = SimpleNamespace(processes_stop_events=processes)
    with pytest.raises(RuntimeError, match='did not exit cleanly'):
        ExperimentRunnerNode.shutdown_all_launch_processes(runner)
    assert calls == [('stop', 1), ('stop', 2), ('join', 1), ('join', 2)]
    assert runner.processes_stop_events == processes


@pytest.mark.parametrize('failed_stage', ['ardupilot', 'launch'])
def test_cleanup_reaches_sessions_after_failure(
        monkeypatch, tmp_path, failed_stage):
    """Attempt session cleanup regardless of earlier teardown failures."""
    terminate = Mock()
    shutdown = Mock()
    failed = terminate if failed_stage == 'ardupilot' else shutdown
    failed.side_effect = RuntimeError('shutdown failed')
    sessions = Mock()
    monkeypatch.setattr('suave_runner.suave_runner.stop_sessions', sessions)
    log = Mock()
    runner = SimpleNamespace(
        ardupilot_proc=Mock(), terminate_process=terminate,
        shutdown_all_launch_processes=shutdown, _active_log_dir=tmp_path,
        _ardupilot_log=log)
    with pytest.raises(RuntimeError, match='shutdown failed'):
        ExperimentRunnerNode.shutdown_all_processes(runner)
    shutdown.assert_called_once_with()
    sessions.assert_called_once_with(tmp_path)
    log.close.assert_called_once_with()


@pytest.mark.parametrize('missing', [False, True])
def test_ardupilot_group_cleanup_does_not_depend_on_live_parent(
        monkeypatch, missing):
    """Stop surviving group members and reap even an exited leader."""
    killpg = Mock(side_effect=ProcessLookupError if missing else None)
    monkeypatch.setattr('suave_runner.suave_runner.os.killpg', killpg)
    proc = Mock(pid=12345)
    proc.poll.return_value = 0
    runner = SimpleNamespace(get_logger=Mock(return_value=Mock()))
    ExperimentRunnerNode.terminate_process(runner, proc, 'ArduPilot')
    killpg.assert_called_once_with(proc.pid, signal.SIGKILL)
    proc.wait.assert_called_once_with(timeout=5)


SIGNAL_CAMPAIGN = """
import subprocess
import sys
import threading
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
import suave_runner.suave_runner as module

root = Path(sys.argv[1])

class Campaign(module.ExperimentRunnerNode):
    def __init__(self):
        Node.__init__(self, 'shutdown_test_runner')
        self.terminate_flag = False
        self._mission_done_event = threading.Event()
        self.processes_stop_events = []
        self.ardupilot_proc = None
        self._active_log_dir = root

    def randomize_experiments_configuration(self):
        pass

    def run_experiments(self):
        child_code = (
            'import time; from pathlib import Path; '
            'from suave_runner.attempt_processes import register_session; '
            f'register_session(Path({str(root / "simulation")!r})); '
            'time.sleep(60)')
        child = subprocess.Popen([sys.executable, '-c', child_code])
        try:
            while not (root / 'simulation/session.json').exists():
                assert child.poll() is None
                time.sleep(0.02)
            (root / 'ready').touch()
            while not self.terminate_flag:
                time.sleep(0.02)
        finally:
            (root / 'cleaning').touch()
            # Leave time for another signal while the campaign owns cleanup.
            time.sleep(0.5)
            assert rclpy.ok(), 'ROS shut down before campaign cleanup'
            self.shutdown_all_processes()
            child.wait(timeout=5)
            (root / 'finished').touch()

module.ExperimentRunnerNode = Campaign
module.main()
"""


def _wait_for(path, proc):
    deadline = time.monotonic() + 15
    while not path.exists():
        assert proc.poll() is None, 'runner exited before ' + path.name
        assert time.monotonic() < deadline, (
            'timed out waiting for ' + path.name)
        time.sleep(0.02)


@pytest.mark.parametrize('signum', [signal.SIGINT, signal.SIGTERM])
def test_main_waits_for_campaign_cleanup_on_signals(tmp_path, signum):
    """Keep ROS and the main process alive until isolated children stop."""
    with (tmp_path / 'output.log').open('w') as output:
        proc = subprocess.Popen(
            [sys.executable, '-c', SIGNAL_CAMPAIGN, str(tmp_path)],
            stdout=output, stderr=output, start_new_session=True)
        try:
            _wait_for(tmp_path / 'ready', proc)
            proc.send_signal(signum)
            _wait_for(tmp_path / 'cleaning', proc)
            proc.send_signal(signum)
            assert proc.wait(timeout=15) == 0, (
                tmp_path / 'output.log').read_text()
            assert (tmp_path / 'finished').exists()
            confirm_stopped(tmp_path)
        finally:
            if proc.poll() is None:
                proc.kill()
            proc.wait()
            # Clean isolated fixture sessions even if the regression returns.
            for record in tmp_path.glob('*/session.json'):
                session = json.loads(record.read_text())['session']
                try:
                    os.killpg(session, signal.SIGKILL)
                except ProcessLookupError:
                    pass
