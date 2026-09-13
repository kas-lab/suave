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
#           "package": "suave_planta",
#           "config_file": "config/exp1_runner_config.yml"
#         }
#
# "package" is any installed ROS package (resolved via `ros2 pkg prefix`)
# and "config_file" is the params file path relative to that package's
# share directory.
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

import rclpy
from rclpy.node import Node
import yaml

active_process = None
received_signal = None
signal_count = 0


def timestamp():
    """Return a readable local timestamp."""
    return datetime.now().astimezone().isoformat(timespec='seconds')


def emit(message, log=None):
    """Print a timestamped orchestration message and optionally log it."""
    line = f'[{timestamp()}] {message}'
    print(line, flush=True)
    if log is not None:
        log.write(line + '\n')
        log.flush()


def write_state(state_file, state):
    """Atomically persist campaign state."""
    state['updated_at'] = timestamp()
    temporary_file = state_file.with_name(f'.{state_file.name}.tmp')
    temporary_file.write_text(
        json.dumps(state, indent=2) + '\n', encoding='utf-8')
    os.replace(temporary_file, state_file)


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


def load_state(state_file):
    """Load and minimally validate a prior campaign state file."""
    try:
        state = json.loads(state_file.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(
            f'cannot read state file {state_file}: {error}') from error

    if state.get('schema_version') != 2:
        raise ValueError(f'unsupported state-file schema in {state_file}')
    if not isinstance(state.get('campaigns'), dict):
        raise ValueError(f'invalid campaign data in {state_file}')
    if not isinstance(state.get('campaigns_manifest'), list):
        raise ValueError(f'missing campaigns_manifest in {state_file}')
    return state


def new_state(campaigns):
    """Create state for a fresh batch defined by a campaign list."""
    names = [entry['name'] for entry in campaigns]
    return {
        'schema_version': 2,
        'created_at': timestamp(),
        'updated_at': timestamp(),
        'campaigns_manifest': campaigns,
        'campaign_order': names,
        'campaigns': {name: {'status': 'pending'} for name in names},
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

    check = subprocess.run(
        ['ros2', 'pkg', 'prefix', 'suave_runner'],
        capture_output=True, text=True, check=False,
    )
    if check.returncode != 0:
        return None, "ROS package 'suave_runner' is not discoverable"

    prefixes = {}
    config_files = {}
    for entry in campaigns:
        package = entry['package']
        if package not in prefixes:
            check = subprocess.run(
                ['ros2', 'pkg', 'prefix', package],
                capture_output=True, text=True, check=False,
            )
            if check.returncode != 0:
                return None, f'ROS package {package!r} is not discoverable'
            prefixes[package] = Path(check.stdout.strip())

        config_file = prefixes[package] / 'share' / package / \
            entry['config_file']
        if not config_file.is_file():
            return None, (
                f'missing installed runner config: {config_file}; '
                'rebuild the workspace')
        config_files[entry['name']] = config_file

    return config_files, ''


def expected_runs(config_file):
    """Return the total number of runs declared in a runner config."""
    try:
        config = yaml.safe_load(config_file.read_text(encoding='utf-8'))
        experiments = config[
            '/suave_runner_node']['ros__parameters']['experiments']
        return sum(
            int(json.loads(experiment).get('num_runs', 1))
            for experiment in experiments
        )
    except (OSError, KeyError, TypeError, ValueError,
            json.JSONDecodeError, yaml.YAMLError) as error:
        raise ValueError(
            f'cannot read experiments from {config_file}: {error}') from error


def completed_runs(result_path):
    """Count successful run markers in a runner result directory."""
    return sum(1 for _ in result_path.glob('run_*_*.done'))


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
        state = load_state(state_file)
        return (
            state_file.parent, state_file, state, state['campaigns_manifest'])

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
    state = new_state(campaigns)
    write_state(state_file, state)
    return batch_dir, state_file, state, campaigns


def print_summary(state, log):
    """Print the status of every campaign."""
    emit('Campaign summary:', log)
    for name in state['campaign_order']:
        status = state['campaigns'][name].get('status', 'unknown')
        emit(f'  {name}: {status}', log)


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
        batch_dir, state_file, state, campaigns = prepare_batch(
            batch_dir_value, resume_state_file_value, campaign_strings)
    except ValueError as error:
        print(f'Error: {error}', file=sys.stderr)
        return 2

    config_files, error = resolve_config_files(campaigns)
    if config_files is None:
        print(f'Error: {error}', file=sys.stderr)
        return 2

    try:
        run_counts = {
            name: expected_runs(config_files[name])
            for name in state['campaign_order']
        }
    except ValueError as error:
        print(f'Error: {error}', file=sys.stderr)
        return 2

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    overall_log_file = batch_dir / 'run_batch.log'
    with overall_log_file.open('a', encoding='utf-8') as overall_log:
        emit(f'Batch directory: {batch_dir}', overall_log)
        emit(f'Checkpoint: {state_file}', overall_log)

        for name in state['campaign_order']:
            campaign_state = state['campaigns'][name]
            if campaign_state.get('status') == 'completed':
                emit(f'Skipping completed campaign: {name}', overall_log)
                continue

            if received_signal is not None:
                break

            result_path_value = campaign_state.get('result_path')
            if result_path_value:
                result_path = Path(result_path_value).expanduser().resolve()
            else:
                result_path = (batch_dir / 'campaigns' / name).resolve()

            config_file = config_files[name]
            campaign_state.update({
                'status': 'running',
                'started_at': timestamp(),
                'finished_at': None,
                'return_code': None,
                'elapsed_seconds': None,
                'error': None,
                'config_file': str(config_file),
                'result_path': str(result_path),
                'expected_runs': run_counts[name],
                'completed_runs': completed_runs(result_path),
            })
            write_state(state_file, state)
            emit(f'Starting campaign: {name}', overall_log)
            emit(f'Resumable results: {result_path}', overall_log)

            log_name = f'{name}.log'
            try:
                return_code, elapsed = run_campaign(
                    config_file, result_path, batch_dir / log_name)
            except OSError as error:
                campaign_state.update({
                    'status': 'failed',
                    'finished_at': timestamp(),
                    'error': str(error),
                    'completed_runs': completed_runs(result_path),
                })
                write_state(state_file, state)
                emit(
                    f'Could not run campaign {name}: {error}',
                    overall_log,
                )
                if fail_fast:
                    break
                continue

            campaign_state.update({
                'finished_at': timestamp(),
                'return_code': return_code,
                'elapsed_seconds': round(elapsed, 3),
                'completed_runs': completed_runs(result_path),
            })

            if received_signal is not None:
                campaign_state['status'] = 'interrupted'
                write_state(state_file, state)
                emit(f'Interrupted during campaign: {name}', overall_log)
                break

            all_runs_completed = (
                campaign_state['completed_runs'] ==
                campaign_state['expected_runs'])
            if return_code == 0 and all_runs_completed:
                campaign_state['status'] = 'completed'
                write_state(state_file, state)
                emit(
                    f'Completed campaign: {name} ({elapsed / 3600:.2f} h)',
                    overall_log,
                )
                continue

            campaign_state['status'] = (
                'incomplete' if return_code == 0 else 'failed')
            write_state(state_file, state)
            if return_code == 0:
                emit(
                    f'Campaign incomplete: {campaign_state["completed_runs"]}'
                    f'/{campaign_state["expected_runs"]} successful runs',
                    overall_log,
                )
            else:
                emit(
                    f'Campaign failed with exit code {return_code}: {name}',
                    overall_log,
                )
            if return_code != 0 and fail_fast:
                break

        print_summary(state, overall_log)

        if received_signal is not None:
            emit(
                'Batch interrupted; resume with: ros2 run suave_runner '
                f'run_batch --ros-args -p resume_state_file:={state_file}',
                overall_log,
            )
            return 128 + received_signal

        incomplete_campaigns = [
            name for name in state['campaign_order']
            if state['campaigns'][name].get('status') == 'incomplete'
        ]
        failed_campaigns = [
            name for name in state['campaign_order']
            if state['campaigns'][name].get('status') == 'failed'
        ]
        pending_campaigns = [
            name for name in state['campaign_order']
            if state['campaigns'][name].get('status') == 'pending'
        ]
        if incomplete_campaigns or failed_campaigns or pending_campaigns:
            emit(
                'Batch finished with '
                f'{len(incomplete_campaigns)} incomplete, '
                f'{len(failed_campaigns)} failed, and '
                f'{len(pending_campaigns)} pending campaign(s).',
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
