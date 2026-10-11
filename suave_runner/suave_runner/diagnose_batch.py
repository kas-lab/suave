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
import re
import sys

RUN = re.compile(r'run_\d+_\d+')
TERMINAL = {'completed', 'failed', 'timeout', 'interrupted'}


def classify(reason):
    """Classify observed failures without asserting an unobserved cause."""
    if 'InvalidHandle' in reason and 'mission_metrics' in reason:
        return 'mission_metrics_invalid_handle'
    if 'change_mode service not available' in reason:
        return 'missing_change_mode_service'
    if 'Timeout waiting for mission_metrics/done' in reason:
        return 'mission_done_timeout'
    if 'did not exit cleanly' in reason:
        return 'launch_process_unclean_exit'
    if 'exited with code' in reason or 'process has died' in reason:
        return 'process_died_before_completion'
    return 'unknown'


def parse_campaign_log(path):
    """Recover attempt history from legacy runner logs, including resumes."""
    attempts = []
    current = None
    exp_idx, manager = None, 'unknown'
    if not path.is_file():
        return attempts
    with path.open(errors='replace') as stream:
        for number, line in enumerate(stream, 1):
            match = re.search(
                r'Running experiment (\d+)/\d+ for \d+ runs with (\S+)', line)
            if match:
                exp_idx, manager = int(match[1]) - 1, match[2]
            match = re.search(r'\]:\s+Run (\S+) (\d+)/\d+\s*$', line)
            if match and exp_idx is not None:
                stamp = re.search(r'\[(\d+\.\d+)\]', line)
                current = {
                    'run_id': f'run_{exp_idx}_{int(match[2]) - 1}',
                    'adaptation_manager': manager, 'status': 'unfinished',
                    'failure_type': '', 'failure_reason': '',
                    'started_epoch': float(stamp[1]) if stamp else None,
                    'evidence': f'{path}:{number}',
                }
                attempts.append(current)
            if current is None:
                continue
            if any(text in line for text in (
                    'Timeout waiting for mission_metrics/done',
                    'Run failed before mission completion:',
                    'Failed to launch processes:', 'flagged as broken')):
                current.update(failure_reason=line.strip(),
                               failure_type=classify(line),
                               evidence=f'{path}:{number}')
            if 'did not exit cleanly' in line:
                current.setdefault('warnings', []).append({
                    'type': 'launch_process_unclean_exit',
                    'evidence': f'{path}:{number}', 'text': line.strip()})
            match = re.search(r'completed \(success: (True|False)\)', line)
            if match:
                current['status'] = (
                    'completed' if match[1] == 'True' else 'failed')
                if match[1] == 'False' and not current['failure_type']:
                    current['failure_type'] = 'unknown'
                current = None
    return attempts


def failure_evidence(log_dir, attempt):
    """Find pre-teardown evidence in the matching attempt's launch logs."""
    evidence = []
    for name in ('experiment', 'simulation'):
        path = log_dir / name / 'launch.log'
        if not path.is_file():
            continue
        with path.open(errors='replace') as stream:
            for number, line in enumerate(stream, 1):
                try:
                    stamp = float(line.split()[0])
                except (ValueError, IndexError):
                    continue
                start = attempt.get('started_epoch')
                end = attempt.get('finished_epoch')
                if start is not None and stamp < start:
                    continue
                if end is not None and stamp > end:
                    break
                if "sending signal 'SIGINT'" in line:
                    break
                kind = classify(line)
                if kind in ('mission_metrics_invalid_handle',
                            'missing_change_mode_service'):
                    if not any(item['type'] == kind for item in evidence):
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


def diagnose_campaign(path, expected=None, log_path=None):
    """Combine sidecars, markers and legacy evidence for one campaign."""
    path = Path(path)
    if log_path is None:
        log_path = path.parent.parent / f'{path.name}.log'
    history = parse_campaign_log(Path(log_path))
    latest = {}
    for attempt in history:
        previous = latest.get(attempt['run_id'])
        if previous:
            previous['finished_epoch'] = attempt.get('started_epoch')
        latest[attempt['run_id']] = attempt
    warnings = []
    for sidecar in sorted(path.glob('run_*_*.status.json')):
        try:
            record = json.loads(sidecar.read_text())
            if not isinstance(record, dict) or not RUN.fullmatch(
                    record.get('run_id', '')) or 'status' not in record:
                raise ValueError('invalid status record')
            latest[record['run_id']] = record
        except (ValueError, OSError) as error:
            warnings.append(f'{sidecar}: {error}')
    for folder in sorted((path / 'logs').glob('run_*_*')):
        if RUN.fullmatch(folder.name):
            latest.setdefault(folder.name, {
                'run_id': folder.name, 'adaptation_manager': 'unknown',
                'status': 'unfinished', 'failure_type': 'unknown',
                'failure_reason': 'No terminal status recorded',
            })
    for marker in path.glob('run_*_*.done'):
        record = latest.setdefault(marker.stem, {
            'run_id': marker.stem, 'adaptation_manager': 'unknown'})
        record.update(status='completed', failure_type='', failure_reason='')
    for run_id, record in latest.items():
        log_dir = Path(record.get('log_dir', ''))
        if not log_dir.is_absolute():
            log_dir = path / log_dir if log_dir.parts else path / 'logs' / run_id
        if record.get('schema_version') != 2:
            log_dir = path / 'logs' / run_id
        record['log_dir'] = str(log_dir)
        if record['status'] in ('failed', 'timeout'):
            record['suspected_causes'] = failure_evidence(log_dir, record)
    rows = {}
    for table in path.glob('*.csv'):
        with table.open(errors='replace', newline='') as stream:
            reader = csv.DictReader(stream)
            if 'mission duration (s)' in (reader.fieldnames or []):
                rows[table.name] = sum(1 for _ in reader)
    records = sorted(latest.values(), key=lambda r: tuple(
        int(i) for i in r['run_id'].split('_')[1:]))
    legacy_failures = Counter(a['run_id'] for a in history
                              if a['status'] == 'failed')
    for record in records:
        count = sum(a.get('status') in ('failed', 'timeout') for a in
                    record.get('attempts', []) + [record])
        legacy_failures[record['run_id']] = max(
            count, legacy_failures[record['run_id']])
    return {
        'path': str(path), 'expected_runs': expected,
        'completed_runs': sum(r['status'] == 'completed' for r in records),
        'historical_failed_attempts': sum(legacy_failures.values()),
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
        campaigns = []
        for name, entry in state['campaigns'].items():
            # Prefer artifacts alongside the checkpoint: container absolute
            # paths often do not exist when diagnosing a mounted host folder.
            folder = state_file.parent / 'campaigns' / name
            if not folder.exists() and entry.get('result_path'):
                folder = Path(entry['result_path']).expanduser()
            campaigns.append(diagnose_campaign(
                folder, entry.get('expected_runs'),
                state_file.parent / f'{name}.log'))
        return {'schema_version': 1, 'campaigns': campaigns}
    if not (path / 'logs').is_dir() and not list(path.glob('run_*.done')):
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
            for warning in record.get('warnings', []):
                yield f'  {record["run_id"]}: teardown warning: {warning}'
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
