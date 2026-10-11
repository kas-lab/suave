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

"""Run a sequence of suave_runner experiment campaigns from ROS parameters."""

# Configured through a regular ROS params file, e.g.:
#
# /run_batch_node:
#   ros__parameters:
#     batch_dir: ''            # optional; auto-generated when empty
#     resume_state_file: ''    # set to resume instead of starting a batch
#     fail_fast: false
#     dry_run: false
#     campaigns: # list of JSON-encoded {name, package, config_file} entries
#       - |
#         {
#           "name": "exp1",
#           "package": "suave_runner",
#           "config_file": "config/runner/exp1_runner_config.yml"
#         }
#
# "package" is any installed ROS package (resolved via the ament index),
# so a campaign can also point at a runner config from a third-party
# managing system's own package, and "config_file" is the params file path
# relative to that package's share directory.
#
# Usage:
#   ros2 run suave_runner run_batch --ros-args --params-file batch.yml
#   ros2 run suave_runner run_batch --ros-args -p resume_state_file:=<path>

from datetime import datetime
import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
import rclpy
from rclpy.node import Node
from suave_runner.diagnose_batch import campaign_progress
from suave_runner.diagnose_batch import expected_runs
from suave_runner.diagnose_batch import summary_lines
from suave_tools.runs import atomic_write_json
from suave_tools.runs import load_runs
from suave_tools.runs import RUNS_FILE

active_process = None
received_signal = None
signal_count = 0


def timestamp():
    """Return a readable local timestamp."""
    return datetime.now().astimezone().isoformat(timespec='seconds')


def emit(message, log=None):
    """Print a timestamped orchestration message and optionally log it."""
    line = f'[{timestamp()}] {message}'
    display = line
    if message.startswith('SYSTEMIC:') and sys.stdout.isatty():
        display = f'\033[31m{line}\033[0m'
    print(display, flush=True)
    if log is not None:
        log.write(line + '\n')
        log.flush()


def parse_campaigns(campaign_strings):
    """Parse and validate the JSON-encoded entries of a campaigns param."""
    if not campaign_strings or list(campaign_strings) == ['']:
        raise ValueError(
            "the 'campaigns' parameter is required and must list at "
            'least one JSON-encoded {name, package, config_file} entry')

    campaigns = []
    names_seen = set()
    for entry_str in campaign_strings:
        try:
            entry = json.loads(entry_str)
        except json.JSONDecodeError as error:
            raise ValueError(
                f'invalid JSON in campaigns parameter entry: {error}') \
                from error
        if not isinstance(entry, dict):
            raise ValueError(f'invalid campaign entry: {entry!r}')
        missing = [
            key for key in ('name', 'package', 'config_file')
            if key not in entry
        ]
        if missing:
            raise ValueError(f'campaign entry missing {missing}: {entry!r}')
        if entry['name'] in names_seen:
            raise ValueError(f'duplicate campaign name {entry["name"]!r}')
        names_seen.add(entry['name'])
        campaigns.append(entry)
    return campaigns


def load_manifest(state_file):
    """Load and minimally validate a batch manifest (state.json)."""
    try:
        manifest = json.loads(state_file.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(
            f'cannot read state file {state_file}: {error}') from error

    if manifest.get('schema_version') != 2:
        raise ValueError(f'unsupported state-file schema in {state_file}')
    if not isinstance(manifest.get('campaigns'), list):
        raise ValueError(f'invalid campaign list in {state_file}')
    return manifest


def new_manifest(campaigns):
    """Describe a batch once; its progress is derived from each campaign."""
    return {
        'schema_version': 2,
        'campaigns': [
            {key: entry[key] for key in ('name', 'package', 'config_file')}
            for entry in campaigns
        ],
    }


def signal_handler(signum, _frame):
    """Forward termination to the complete active ros2 process group."""
    global received_signal, signal_count
    signal_count += 1
    if received_signal is None:
        received_signal = signum

    process = active_process
    if process is None or process.poll() is not None:
        return

    forwarded_signal = signum if signal_count == 1 else signal.SIGKILL
    try:
        os.killpg(process.pid, forwarded_signal)
    except ProcessLookupError:
        pass


def resolve_config_files(campaigns):
    """Resolve each campaign's installed config file via its ROS package."""
    if shutil.which('ros2') is None:
        return None, 'ros2 is unavailable; source the ROS workspace first'

    config_files = {}
    for entry in campaigns:
        package = entry['package']
        try:
            share = Path(get_package_share_directory(package))
        except PackageNotFoundError:
            return None, f'ROS package {package!r} is not discoverable'

        config_file = share / entry['config_file']
        if not config_file.is_file():
            return None, (
                f'missing installed runner config: {config_file}; '
                'rebuild the workspace')
        config_files[entry['name']] = config_file

    return config_files, ''


def run_campaign(config_file, result_path, log_file):
    """Run one runner config and mirror its combined output into a log."""
    global active_process
    command = [
        'ros2', 'run', 'suave_runner', 'suave_runner',
        '--ros-args',
        '--params-file', str(config_file),
        '-p', f'resume_result_path:={result_path}',
    ]
    started_at = time.monotonic()
    process = None

    try:
        with log_file.open('a', encoding='utf-8') as campaign_log:
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding='utf-8',
                errors='replace',
                bufsize=1,
                start_new_session=True,
            )
            active_process = process
            if received_signal is not None:
                os.killpg(process.pid, received_signal)
            assert process.stdout is not None
            for line in process.stdout:
                sys.stdout.write(line)
                sys.stdout.flush()
                campaign_log.write(line)
                campaign_log.flush()
            return_code = process.wait()
    finally:
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        active_process = None

    return return_code, time.monotonic() - started_at


def read_batch_parameters(args=None):
    """Read batch parameters from ROS args/params file via a plain node."""
    rclpy.init(args=args)
    node = Node('run_batch_node')
    node.declare_parameter('batch_dir', '')
    node.declare_parameter('resume_state_file', '')
    node.declare_parameter('fail_fast', False)
    node.declare_parameter('dry_run', False)
    # List of JSON-encoded {name, package, config_file} dicts.
    node.declare_parameter('campaigns', [''])

    parameters = {
        'batch_dir': node.get_parameter(
            'batch_dir').get_parameter_value().string_value,
        'resume_state_file': node.get_parameter(
            'resume_state_file').get_parameter_value().string_value,
        'fail_fast': node.get_parameter(
            'fail_fast').get_parameter_value().bool_value,
        'dry_run': node.get_parameter(
            'dry_run').get_parameter_value().bool_value,
        'campaign_strings': list(node.get_parameter(
            'campaigns').get_parameter_value().string_array_value),
    }

    node.destroy_node()
    rclpy.shutdown()
    return parameters


def prepare_batch(batch_dir_value, resume_state_file_value, campaign_strings):
    """Create a new batch directory or load an existing checkpoint."""
    if batch_dir_value and resume_state_file_value:
        raise ValueError(
            "set only one of 'batch_dir' or 'resume_state_file'")

    if resume_state_file_value:
        state_file = Path(
            resume_state_file_value).expanduser().resolve()
        if not state_file.is_file():
            raise ValueError(f'state file does not exist: {state_file}')
        manifest = load_manifest(state_file)
        return state_file.parent, state_file, manifest['campaigns']

    campaigns = parse_campaigns(campaign_strings)

    if batch_dir_value:
        batch_dir = Path(batch_dir_value).expanduser().resolve()
    else:
        name = datetime.now().strftime('batch_%Y%m%d_%H%M%S')
        batch_dir = (Path('~/suave/results/batches').expanduser() / name)

    if batch_dir.exists():
        raise ValueError(
            f'batch directory already exists: {batch_dir}; resume it with '
            "the 'resume_state_file' parameter")
    batch_dir.mkdir(parents=True)
    state_file = batch_dir / 'state.json'
    atomic_write_json(state_file, new_manifest(campaigns))
    return batch_dir, state_file, campaigns


def progress(batch_dir, name, config_file):
    """Return (completed, expected) runs, or None if runs.json is unreadable."""
    try:
        return campaign_progress(batch_dir / 'campaigns' / name, config_file)
    except (OSError, ValueError, KeyError, TypeError):
        return None


def campaign_done(batch_dir, name, config_file):
    """Return whether every run a campaign's config declares has completed."""
    counts = progress(batch_dir, name, config_file)
    return counts is not None and counts[0] == counts[1]


def print_summary(batch_dir, config_files, log):
    """Print each campaign's progress, derived from its runs.json."""
    emit('Campaign summary:', log)
    for name, config_file in config_files.items():
        counts = progress(batch_dir, name, config_file)
        text = 'unreadable runs.json' if counts is None else (
            f'{counts[0]}/{counts[1]} runs completed')
        emit(f'  {name}: {text}', log)


def emit_run_summary(result_path, log):
    """Print per-manager outcomes; a damaged runs.json must not stop a batch."""
    if not (result_path / RUNS_FILE).exists():
        return
    try:
        runs = load_runs(result_path)['runs']
    except (OSError, ValueError) as error:
        emit(f'Cannot read {result_path / RUNS_FILE}: {error}', log)
        return
    for line in summary_lines(runs.values()):
        emit(line, log)


def run_dry(campaign_strings):
    """Print the launch commands a batch would run without executing them."""
    campaigns = parse_campaigns(campaign_strings)
    for entry in campaigns:
        config_file = (
            Path(f'<{entry["package"]}_share>') / entry['config_file'])
        result_path = Path('<batch_dir>') / 'campaigns' / entry['name']
        command = [
            'ros2', 'run', 'suave_runner', 'suave_runner',
            '--ros-args',
            '--params-file', str(config_file),
            '-p', f'resume_result_path:={result_path}',
        ]
        print(shlex.join(command))
    return 0


def run_batch(batch_dir_value, resume_state_file_value, campaign_strings,
              fail_fast):
    """Run or resume the complete experiment batch."""
    try:
        batch_dir, state_file, campaigns = prepare_batch(
            batch_dir_value, resume_state_file_value, campaign_strings)
    except ValueError as error:
        print(f'Error: {error}', file=sys.stderr)
        return 2

    config_files, error = resolve_config_files(campaigns)
    if config_files is None:
        print(f'Error: {error}', file=sys.stderr)
        return 2

    try:
        for config_file in config_files.values():
            expected_runs(config_file)
    except ValueError as error:
        print(f'Error: {error}', file=sys.stderr)
        return 2

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    overall_log_file = batch_dir / 'run_batch.log'
    with overall_log_file.open('a', encoding='utf-8') as overall_log:
        emit(f'Batch directory: {batch_dir}', overall_log)
        emit(f'Manifest: {state_file}', overall_log)

        for name, config_file in config_files.items():
            if campaign_done(batch_dir, name, config_file):
                emit(f'Skipping completed campaign: {name}', overall_log)
                continue

            if received_signal is not None:
                break

            result_path = (batch_dir / 'campaigns' / name).resolve()
            emit(f'Starting campaign: {name}', overall_log)
            emit(f'Resumable results: {result_path}', overall_log)

            log_name = f'{name}.log'
            try:
                return_code, elapsed = run_campaign(
                    config_file, result_path, batch_dir / log_name)
            except OSError as error:
                emit(
                    f'Could not run campaign {name}: {error}',
                    overall_log,
                )
                if fail_fast:
                    break
                continue

            if received_signal is not None:
                emit(f'Interrupted during campaign: {name}', overall_log)
                break

            emit(f'Run summary for {name}:', overall_log)
            emit_run_summary(result_path, overall_log)
            print_summary(batch_dir, config_files, overall_log)

            if return_code == 0 and campaign_done(
                    batch_dir, name, config_file):
                emit(
                    f'Completed campaign: {name} ({elapsed / 3600:.2f} h)',
                    overall_log,
                )
                continue

            counts = progress(batch_dir, name, config_file)
            if return_code == 0 and counts is not None:
                emit(
                    f'Campaign incomplete: {counts[0]}/{counts[1]} '
                    'successful runs',
                    overall_log,
                )
            elif return_code == 0:
                emit(f'Campaign {name} has an unreadable runs.json',
                     overall_log)
            else:
                emit(
                    f'Campaign failed with exit code {return_code}: {name}',
                    overall_log,
                )
            if return_code != 0 and fail_fast:
                break

        print_summary(batch_dir, config_files, overall_log)

        if received_signal is not None:
            emit(
                'Batch interrupted; resume with: ros2 run suave_runner '
                f'run_batch --ros-args -p resume_state_file:={state_file}',
                overall_log,
            )
            return 128 + received_signal

        unfinished = [
            name for name, config_file in config_files.items()
            if not campaign_done(batch_dir, name, config_file)]
        if unfinished:
            emit(
                f'Batch finished with {len(unfinished)} unfinished '
                f'campaign(s): {", ".join(unfinished)}.',
                overall_log,
            )
            emit(
                'Retry unfinished runs with: ros2 run suave_runner '
                f'run_batch --ros-args -p resume_state_file:={state_file}',
                overall_log,
            )
            return 1

        emit('All campaigns completed successfully.', overall_log)
        return 0


def main(args=None):
    """Read batch parameters and run or resume the campaign sequence."""
    parameters = read_batch_parameters(args)

    if parameters['dry_run']:
        return run_dry(parameters['campaign_strings'])

    return run_batch(
        parameters['batch_dir'],
        parameters['resume_state_file'],
        parameters['campaign_strings'],
        parameters['fail_fast'],
    )


if __name__ == '__main__':
    sys.exit(main())
