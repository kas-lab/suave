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

"""Finalize a real MCAP recorder through the runner's launch lifecycle."""

import functools
import multiprocessing
import shutil
import time
from types import SimpleNamespace

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import pytest
from suave_runner.attempt_processes import stop_sessions
from suave_runner.bag_validation import validate_bag
from suave_runner.run_artifacts import attempt_details
from suave_runner.run_artifacts import finalize_attempt
from suave_runner.run_status import open_runs
from suave_runner.run_status import start_attempt
from suave_runner.suave_runner import ExperimentRunnerNode
from suave_tools.runs import accepted_runs
from suave_tools.runs import bag_dir
from suave_tools.runs import load_runs
from suave_tools.runs import log_dir as attempt_log_dir
from suave_tools.runs import write_runs

TOPIC = '/suave_recording_integration'


def _launch_runner():
    runner = SimpleNamespace(
        _record_process_exit=ExperimentRunnerNode._record_process_exit,
        _process_failure_queue=multiprocessing.Queue(),
        processes_stop_events=[])
    runner._run_launchfile = functools.partial(
        ExperimentRunnerNode._run_launchfile, runner)
    return runner


def _recording(bag):
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-s', 'mcap',
                 '--storage-preset-profile', 'fastwrite',
                 '-o', str(bag), TOPIC],
            name='rosbag_record', sigterm_timeout='20'),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-r', '20', TOPIC,
                 'std_msgs/msg/String', '{data: x}'],
            name='publisher'),
    ])


@pytest.mark.skipif(shutil.which('ros2') is None, reason='ros2 unavailable')
def test_real_recorder_is_stopped_validated_and_checkpointed(
        tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '87')
    runs = open_runs(tmp_path)
    details = attempt_details('run_0_0', 'bt', 'suave', True)
    entry = start_attempt(
        runs, 'run_0_0', 'bt', 'suave', 'ros2 launch pkg file', **details)
    write_runs(tmp_path, runs)
    log_dir = attempt_log_dir(tmp_path, 'run_0_0', entry)
    bag = bag_dir(tmp_path, 'run_0_0', entry, 'pending')
    bag.parent.mkdir(parents=True)

    runner = _launch_runner()
    process, stop_event = ExperimentRunnerNode.start_launch_process(
        runner, _recording(bag), log_dir / 'experiment', 'integration')
    runner.processes_stop_events.append((process, stop_event, 'integration'))
    deadline = time.monotonic() + 30
    while not bag.exists() and time.monotonic() < deadline:
        time.sleep(0.2)
    assert bag.exists(), 'recorder never created its output directory'
    time.sleep(5)

    ExperimentRunnerNode.shutdown_all_launch_processes(runner)
    stop_sessions(log_dir)
    assert runner._process_failure_queue.empty()

    finalize_attempt(tmp_path, runs, 'run_0_0', 'completed', validate_bag)

    stored = load_runs(tmp_path)['runs']['run_0_0']
    assert stored['status'] == 'completed'
    assert stored['bag'] == 'accepted'
    assert not bag.exists()
    accepted = bag_dir(tmp_path, 'run_0_0', stored)
    assert validate_bag(accepted)['message_count'] > 0
    entries, _ = accepted_runs(tmp_path)
    assert [item['bag_path'] for item in entries] == [
        str(accepted.relative_to(tmp_path.resolve()))]
