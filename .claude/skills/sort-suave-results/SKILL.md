---
name: sort-suave-results
description: Use when a SUAVE experiment result CSV (under suave_runner) may contain resumed/retried runs appended out of order, and the true per-run ordering needs to be restored before pairing or statistical analysis. Points to the deterministic sort_results.py script; never reorders rows by hand or by eyeballing timestamps/positions.
---

# Sort SUAVE Results

`--resume`d SUAVE campaigns (`run_all_experiments.py --resume`, or `suave_runner`'s
own checkpoint skip) append retried runs to the end of a managing system's
result CSV, out of their original run order. That breaks any pairing that
assumes row order reflects run order (e.g. per-run comparisons across
managing systems, or a paired/Wilcoxon analysis).

Do not reorder rows by hand, by eyeballing `initial pos (x,y)`, or by
assuming retried rows landed in a particular block. Always use
`suave_runner/sort_results.py` (in `src/suave/suave_runner/suave_runner/`) --
it deterministically recovers the true `run_idx` for every row, and refuses
(raises, does not silently guess) whenever the reconstruction can't be
trusted.

## How it works

Every run also touches a `run_{exp_idx}_{run_idx}.done` marker the moment it
completes, in the same result directory as the CSVs, named with the
authoritative run index for that managing system (`exp_idx` is the managing
system's position in the campaign's `experiments` list, e.g. from
`suave_runner/config/runner/exp1_runner_config.yml`; `run_idx` is 0-based within
that system's runs). The script matches each CSV row's `datetime` to the
nearest `.done` marker's modification time, in ascending order on both
sides, and writes a new CSV sorted by the recovered `run_idx`.

It never overwrites the source CSV. It raises `ValueError` instead of
guessing when:
- the row count and `.done` marker count differ for that `exp_idx`;
- any matched row/marker pair is more than 120 seconds apart (a real match
  lands within ~16s, per the runner's `time.sleep(10)` between runs);
- the recovered `run_idx` values are not an exact permutation of
  `0..n-1`.

## Usage

Single CSV:

```bash
python3 src/suave/suave_runner/suave_runner/sort_results.py \
  --csv /path/to/campaigns/exp1/bt_suave.csv \
  --exp-idx 2 \
  --output /path/to/campaigns/exp1/sorted/bt_suave_sorted.csv
```

`--exp-idx` is that managing system's index in the campaign's `experiments`
list (check the matching `*_runner_config.yml`, e.g. for `exp1`:
`bt=0, metacontrol=1, random=2, none=3`. `--done-dir`
defaults to the CSV's own directory and rarely needs to be set explicitly.

Whole campaign at once, via a JSON manifest (mirrors the `data_files`
convention used in `suave_runner/config/analysis/*_analysis_config.yml`):

```json
[
  {"managing_system": "bt", "data_file": "/path/campaigns/exp1/bt_suave.csv", "exp_idx": 0},
  {"managing_system": "metacontrol", "data_file": "/path/campaigns/exp1/metacontrol_suave.csv", "exp_idx": 1},
  {"managing_system": "random", "data_file": "/path/campaigns/exp1/random_suave.csv", "exp_idx": 2},
  {"managing_system": "none", "data_file": "/path/campaigns/exp1/none_suave.csv", "exp_idx": 3}
]
```

A third-party managing system's own runner config can add more entries the
same way, from its own repo -- `exp_idx` just needs to match that system's
position in whichever campaign's `experiments` list actually launched it.

```bash
python3 src/suave/suave_runner/suave_runner/sort_results.py \
  --manifest manifest.json \
  --output /path/campaigns/exp1/sorted/
```

Output CSVs gain a prepended `run_idx` column and are sorted ascending by it;
every other column is unchanged.

## After sorting

Each system's sorted CSV now has `run_idx` as a genuine per-run pairing key
across managing systems within the same campaign -- runs share the same
`run_idx` because the campaign's perturbation config (initial position,
water-visibility shift, thruster-failure timing) is generated once per local
run index and reused across managing systems (see
`suave_runner.py`'s `randomize_experiments_configuration`). Do not assume
`run_idx` is comparable *across different campaigns* (`exp1` vs `exp3`, etc.)
-- each campaign draws its own perturbation sequence.

If the script raises an error, report it as-is rather than working around
it (e.g. by trimming rows to force equal counts) -- it means the `.done`
markers or CSV for that system/campaign don't support a safe reconstruction,
and that must be investigated before any paired analysis is trusted.
