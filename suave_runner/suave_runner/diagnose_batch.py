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

"""Read-only, ROS-independent diagnosis of batch and campaign artifacts."""

import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from suave_tools.runs import load_runs
from suave_tools.runs import log_dir as attempt_log_dir
from suave_tools.runs import parse_run_id
from suave_tools.runs import RUNS_FILE
import yaml

TERMINAL = {'completed', 'failed', 'interrupted'}


def classify(line):
    """Classify an observed failure cause without asserting an unseen one."""
    if 'InvalidHandle' in line and 'mission_metrics' in line:
        return 'mission_metrics_invalid_handle'
    if 'change_mode service not available' in line:
        return 'missing_change_mode_service'
    return ''


def failure_evidence(log_dir):
    """Find pre-teardown evidence in an attempt's launch logs."""
    evidence = []
    for name in ('experiment', 'simulation'):
        path = log_dir / name / 'launch.log'
        if not path.is_file():
            continue
        with path.open(errors='replace') as stream:
            for number, line in enumerate(stream, 1):
                if "sending signal 'SIGINT'" in line:
                    break
                kind = classify(line)
                if kind and not any(item['type'] == kind for item in evidence):
                    evidence.append({'type': kind, 'log': str(path),
                                     'line': number, 'text': line.strip()})
    return evidence


def summarize(records):
    """Summarize latest outcomes by manager with systemic warnings."""
    groups = {}
    for record in records:
        manager = record.get('adaptation_manager', 'unknown')
        group = groups.setdefault(manager, {
            'attempted': 0, 'completed': 0, 'failures': Counter(),
            'systemic': [],
        })
        if record['status'] not in TERMINAL:
            continue
        group['attempted'] += 1
        if record['status'] == 'completed':
            group['completed'] += 1
        elif record['status'] != 'interrupted':
            group['failures'][record.get('failure_type') or 'unknown'] += 1
    for group in groups.values():
        for kind, count in group['failures'].items():
            # A single failure is insufficient evidence of persistence.
            if count >= 3 or (count >= 2 and count > group['attempted'] / 2):
                group['systemic'].append(kind)
        group['failures'] = dict(group['failures'])
    return groups


def summary_lines(records):
    """Render manager summaries for both offline and live runner output."""
    for manager, group in summarize(records).items():
        yield (f'{manager}: {group["completed"]}/{group["attempted"]} '
               f'completed; failures={group["failures"]}')
        for kind in group['systemic']:
            yield f'SYSTEMIC: {manager}: repeated {kind}'


def expected_runs(config_file):
    """Return the total number of runs declared in a runner config."""
    try:
        config = yaml.safe_load(Path(config_file).read_text(encoding='utf-8'))
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


def completed_runs(campaign):
    """Count completed runs in runs.json; a campaign not yet started has 0."""
    if not (Path(campaign) / RUNS_FILE).exists():
        return 0
    return sum(entry['status'] == 'completed'
               for entry in load_runs(campaign)['runs'].values())


def campaign_progress(campaign, config_file):
    """Return (completed, expected) runs; done iff the two are equal."""
    return completed_runs(campaign), expected_runs(config_file)


def diagnose_campaign(path, expected=None):
    """Report a campaign's runs.json with failure evidence from its logs."""
    path = Path(path)
    warnings, records = [], []
    try:
        runs = load_runs(path)['runs']
    except (ValueError, OSError) as error:
        # Schema errors name the file; JSON and OS errors do not.
        message = str(error)
        if str(path) not in message:
            message = f'{path / RUNS_FILE}: {message}'
        warnings.append(message)
        runs = {}
    for run_id in sorted(runs, key=parse_run_id):
        record = dict(runs[run_id], run_id=run_id)
        if record['status'] == 'completed' and record['bag'] not in (
                None, 'accepted'):
            warnings.append(
                f'{run_id}: completed with bag {record["bag"]}')
        try:
            log_dir = attempt_log_dir(path, run_id, record)
        except ValueError as error:
            warnings.append(f'{run_id}: {error}')
            continue
        record['log_dir'] = str(log_dir)
        if record['status'] == 'failed':
            record['suspected_causes'] = failure_evidence(log_dir)
        records.append(record)
    rows = {}
    for table in path.glob('*.csv'):
        with table.open(errors='replace', newline='') as stream:
            reader = csv.DictReader(stream)
            if 'mission duration (s)' in (reader.fieldnames or []):
                rows[table.name] = sum(1 for _ in reader)
    return {
        'path': str(path), 'expected_runs': expected,
        'completed_runs': sum(r['status'] == 'completed' for r in records),
        'historical_failed_attempts': sum(
            attempt.get('status') == 'failed'
            for record in records
            for attempt in record['attempts'] + [record]),
        'runs': records, 'managers': summarize(records),
        'csv_rows': rows, 'warnings': warnings,
    }


def _manifest_expected(entry):
    # The manifest names an installed config; without a sourced workspace
    # the expected count is unknown rather than an error.
    try:
        share = Path(get_package_share_directory(entry['package']))
    except (LookupError, OSError):
        return None
    try:
        return expected_runs(share / entry['config_file'])
    except ValueError:
        return None


def diagnose(path):
    """Inspect a batch manifest, batch directory, or campaign directory."""
    path = Path(path).expanduser().resolve()
    if not path.exists():
        raise ValueError(f'not found: {path}')
    state_file = path if path.is_file() else path / 'state.json'
    if state_file.is_file():
        manifest = json.loads(state_file.read_text())
        if manifest.get('schema_version') != 2 or not isinstance(
                manifest.get('campaigns'), list):
            raise ValueError(f'unsupported state-file schema: {state_file}')
        campaigns = [
            diagnose_campaign(state_file.parent / 'campaigns' / entry['name'],
                              _manifest_expected(entry))
            for entry in manifest['campaigns']]
        return {'schema_version': 1, 'campaigns': campaigns}
    if not (path / RUNS_FILE).is_file():
        raise ValueError(f'not a campaign result directory: {path}')
    return {'schema_version': 1, 'campaigns': [diagnose_campaign(path)]}


def render(report):
    """Yield a concise report with actionable evidence locations."""
    for campaign in report['campaigns']:
        yield f'Campaign: {campaign["path"]}'
        yield (f'Completed: {campaign["completed_runs"]}/'
               f'{campaign["expected_runs"] or "unknown"}; historical failed '
               f'attempts: {campaign["historical_failed_attempts"]}')
        yield from summary_lines(campaign['runs'])
        for record in campaign['runs']:
            if record['status'] == 'completed':
                continue
            yield (f'  {record["run_id"]}: {record["status"]} '
                   f'{record.get("failure_type", "")} '
                   f'{record.get("failure_reason", "")}')
            yield f'    logs: {record["log_dir"]}'
            for item in record.get('suspected_causes', []):
                yield (f'    suspected {item["type"]}: '
                       f'{item["log"]}:{item["line"]}')
        yield ('Mission CSV rows (not completion proof): '
               f'{campaign["csv_rows"]}')
        yield from campaign['warnings']


def main(argv=None):
    """Print diagnosis; return 2 for invalid input, not mission failures."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('path', help='state.json, batch or campaign folder')
    parser.add_argument('--json', action='store_true', help='print JSON report')
    args = parser.parse_args(argv)
    try:
        report = diagnose(args.path)
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(2, f'Diagnosis failed: {error}\n')
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        for line in render(report):
            if line.startswith('SYSTEMIC:') and sys.stdout.isatty():
                line = f'\033[31m{line}\033[0m'
            print(line)
    return 0


if __name__ == '__main__':
    sys.exit(main())
