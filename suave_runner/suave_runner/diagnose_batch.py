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

from suave_tools.bag_selection import artifact_path
from suave_tools.bag_selection import load_status

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


def diagnose_campaign(path, expected=None):
    """Combine status sidecars and checkpoint markers for one campaign."""
    path = Path(path)
    latest, warnings = {}, []
    for sidecar in sorted(path.glob('run_*_*.status.json')):
        try:
            record = load_status(sidecar)
        except (ValueError, OSError) as error:
            # Schema errors name the file; JSON and OS errors do not.
            message = str(error)
            if str(sidecar) not in message:
                message = f'{sidecar}: {message}'
            warnings.append(message)
            continue
        latest[record['run_id']] = record
    markers = {marker.stem for marker in path.glob('run_*_*.done')}
    for run_id in sorted(markers - latest.keys()):
        warnings.append(f'{run_id}: checkpoint without a valid status record')
    for run_id, record in latest.items():
        if (record['status'] == 'completed') != (run_id in markers):
            warnings.append(
                f'{run_id}: status {record["status"]} disagrees with '
                'checkpoint')
        try:
            log_dir = artifact_path(path, record['log_dir'])
        except ValueError as error:
            warnings.append(f'{run_id}: {error}')
            continue
        record['log_dir'] = str(log_dir)
        if record['status'] == 'failed':
            record['suspected_causes'] = failure_evidence(log_dir)
    rows = {}
    for table in path.glob('*.csv'):
        with table.open(errors='replace', newline='') as stream:
            reader = csv.DictReader(stream)
            if 'mission duration (s)' in (reader.fieldnames or []):
                rows[table.name] = sum(1 for _ in reader)
    records = sorted(latest.values(), key=lambda r: tuple(
        int(i) for i in r['run_id'].split('_')[1:]))
    return {
        'path': str(path), 'expected_runs': expected,
        'completed_runs': sum(r['status'] == 'completed' and
                              r['run_id'] in markers for r in records),
        'historical_failed_attempts': sum(
            attempt.get('status') == 'failed'
            for record in records
            for attempt in record['attempts'] + [record]),
        'runs': records, 'managers': summarize(records),
        'csv_rows': rows, 'warnings': warnings,
    }


def diagnose(path):
    """Inspect a batch state file, batch directory, or campaign directory."""
    path = Path(path).expanduser().resolve()
    if not path.exists():
        raise ValueError(f'not found: {path}')
    state_file = path if path.is_file() else path / 'state.json'
    if state_file.is_file():
        state = json.loads(state_file.read_text())
        if state.get('schema_version') != 1:
            raise ValueError(f'unsupported state-file schema: {state_file}')
        campaigns = []
        for name, entry in state['campaigns'].items():
            # Prefer artifacts alongside the checkpoint: container absolute
            # paths often do not exist when diagnosing a mounted host folder.
            folder = state_file.parent / 'campaigns' / name
            if not folder.exists() and entry.get('result_path'):
                folder = Path(entry['result_path']).expanduser()
            campaigns.append(diagnose_campaign(
                folder, entry.get('expected_runs')))
        return {'schema_version': 1, 'campaigns': campaigns}
    if not list(path.glob('run_*_*.status.json')):
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
