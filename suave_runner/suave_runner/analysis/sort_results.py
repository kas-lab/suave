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

"""Recover the true run index for SUAVE result rows disrupted by --resume."""

#
# A `--resume`d campaign appends retried runs to the end of each managing
# system's result CSV, out of their original run order. Since every run also
# touches a `run_{exp_idx}_{run_idx}.done` marker the moment it completes (named
# with the authoritative run index, in the same result directory as the CSVs),
# each result row can be matched to its true run index by pairing CSV rows and
# `.done` markers in ascending order of `datetime` / modification time
# respectively.
#
# This module never overwrites a source CSV: it writes a new, sorted copy with
# a prepended `run_idx` column. It refuses (raises `ValueError`) rather than
# guess whenever the reconstruction cannot be trusted -- a row/marker count
# mismatch, a match separated by more than `max_gap` seconds, or a result that
# is not an exact permutation of `0..n-1`.
#
# Run standalone:
#     python3 sort_results.py --csv path/to/bt_suave.csv --exp-idx 2 --output
#     path/to/bt_suave_sorted.csv
#
#     python3 sort_results.py --manifest manifest.json --output sorted_dir/
#
# Manifest format (one entry per managing system, `done_dir` optional):
#     [
#       {"managing_system": "bt", "data_file": "campaigns/exp1/bt_suave.csv",
#        "exp_idx": 2}
#     ]

import argparse
from datetime import datetime
import json
from pathlib import Path
import re

import pandas as pd

DONE_PATTERN = re.compile(r'run_(\d+)_(\d+)\.done$')
MAX_GAP_SECONDS = 120  # sanity threshold; genuine matches land within ~16s


def _parse_datetime(value):
    return datetime.strptime(value, '%d-%b-%Y-%H-%M-%S')


def _done_files(done_dir, exp_idx):
    """Return (run_idx, mtime) for every done marker of one managing system."""
    files = []
    for path in Path(done_dir).glob(f'run_{exp_idx}_*.done'):
        match = DONE_PATTERN.search(path.name)
        run_idx = int(match.group(2))
        files.append((run_idx, datetime.fromtimestamp(path.stat().st_mtime)))
    return files


def reconstruct_run_index(csv_path, done_dir, exp_idx, max_gap=MAX_GAP_SECONDS):
    """Return a DataFrame sorted by reconstructed run_idx."""
    # Raises ValueError whenever the reconstruction cannot be trusted, instead
    # of silently falling back to row order.
    df = pd.read_csv(csv_path)
    if 'datetime' not in df.columns:
        raise ValueError(f"{csv_path} has no 'datetime' column to match on")
    df = df.copy()
    df['_dt'] = df['datetime'].apply(_parse_datetime)

    done = _done_files(done_dir, exp_idx)
    if len(done) != len(df):
        raise ValueError(
            f"{csv_path}: {len(df)} result rows but {len(done)} '.done' "
            f'markers for exp_idx={exp_idx} in {done_dir}; cannot '
            'reconstruct run_idx safely')

    rows_by_time = df.sort_values('_dt').index.tolist()
    done_by_time = sorted(done, key=lambda item: item[1])

    run_idx_by_row = {}
    for row_index, (run_idx, mtime) in zip(rows_by_time, done_by_time):
        gap = abs((df.loc[row_index, '_dt'] - mtime).total_seconds())
        if gap > max_gap:
            raise ValueError(
                f"{csv_path}: row at {df.loc[row_index, 'datetime']} is "
                f"{gap:.0f}s from its nearest '.done' marker (run_idx="
                f'{run_idx}), exceeding the {max_gap}s sanity threshold')
        run_idx_by_row[row_index] = run_idx

    expected = set(range(len(df)))
    actual = set(run_idx_by_row.values())
    if actual != expected:
        raise ValueError(
            f'{csv_path}: reconstructed run_idx values {sorted(actual)} are '
            f'not a permutation of 0..{len(df) - 1}')

    df['run_idx'] = df.index.map(run_idx_by_row)
    df = df.drop(columns=['_dt']).sort_values('run_idx').reset_index(drop=True)
    ordered_columns = ['run_idx'] + [c for c in df.columns if c != 'run_idx']
    return df[ordered_columns]


def process_manifest(manifest_path, output_dir):
    """Sort every system's CSV listed in a JSON manifest into output_dir."""
    manifest = json.loads(Path(manifest_path).expanduser().read_text())
    output_dir = Path(output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)

    written = []
    for entry in manifest:
        csv_path = Path(entry['data_file']).expanduser()
        done_dir = Path(entry.get('done_dir', csv_path.parent)).expanduser()
        sorted_df = reconstruct_run_index(csv_path, done_dir, entry['exp_idx'])
        output_path = output_dir / f'{csv_path.stem}_sorted.csv'
        sorted_df.to_csv(output_path, index=False)
        written.append(output_path)
    return written


def main(args=None):
    """Sort one CSV or every CSV in a manifest by reconstructed run_idx."""
    parser = argparse.ArgumentParser(
        description=(
            'Recover the true run_idx for SUAVE result rows disrupted by '
            '--resume, and write sorted, run_idx-tagged CSV copies. Never '
            'overwrites source CSVs.'))
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        '--manifest',
        help=(
            'JSON file: a list of {"managing_system", "data_file", '
            '"exp_idx"} objects, one per system ("done_dir" optional, '
            "defaults to each data_file's own directory)."))
    group.add_argument('--csv', help='Single result CSV to sort.')
    parser.add_argument(
        '--exp-idx', type=int,
        help=(
            "Managing system's index in that campaign's run_N_M.done "
            'markers (required with --csv).'))
    parser.add_argument(
        '--done-dir',
        help=(
            'Directory containing the run_N_M.done markers (defaults to '
            "the CSV's own directory; only used with --csv)."))
    parser.add_argument(
        '--output', required=True,
        help='Output file (with --csv) or output directory (with '
             '--manifest).')
    parsed = parser.parse_args(args)

    if parsed.manifest:
        for path in process_manifest(parsed.manifest, parsed.output):
            print(f'Wrote {path}')
        return

    if parsed.exp_idx is None:
        parser.error('--exp-idx is required with --csv')
    csv_path = Path(parsed.csv).expanduser()
    done_dir = (
        Path(parsed.done_dir).expanduser() if parsed.done_dir
        else csv_path.parent)
    sorted_df = reconstruct_run_index(csv_path, done_dir, parsed.exp_idx)
    output_path = Path(parsed.output).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    sorted_df.to_csv(output_path, index=False)
    print(f'Wrote {output_path}')


if __name__ == '__main__':
    main()
