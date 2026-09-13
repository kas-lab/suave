---
name: sort-suave-results
description: Recover authoritative run_idx values and ordering for SUAVE campaign CSVs using their completion markers, especially after resumed or retried runs. Use before matched-run analysis when CSVs lack verified run identities.
---

# Sort SUAVE Results

Use the existing `suave_runner/suave_runner/sort_results.py` implementation
to reconstruct run identities. Resumed campaigns can append retried runs
after later runs; CSV position and initial position alone cannot establish
pairing.

## Repository and execution

The PLANTA checkout is normally
`/home/gus/ros_workspaces/planta_ws/src/suave`. Locate the corresponding
SUAVE checkout when working elsewhere. Follow its `AGENTS.md`, including
running SUAVE commands inside the applicable development or integration
container with its default sourced environment. Translate paths through the
container's mounts; the command examples below run from the ROS workspace
root inside that container.

Read `suave_runner/suave_runner/sort_results.py` in that checkout when
checking CLI options or investigating a reconstruction failure. Reuse it
rather than maintaining a second implementation in this skill.

## Recover the run indices

1. Locate the aggregate CSVs, original `run_{exp_idx}_{run_idx}.done`
   markers, and matching campaign configuration.
2. Determine each system's `exp_idx` from that campaign's `experiments`
   list. Indices vary between campaigns; do not infer them from a system
   name or copy another campaign's mapping.
3. Choose output paths distinct from all input CSVs. Preserve the raw data
   and original marker modification times.
4. Run the sorter for one CSV or a manifest.

Single CSV (`--exp-idx 2` is illustrative):

```bash
python3 src/suave/suave_runner/suave_runner/sort_results.py \
  --csv /path/campaign/bt_suave.csv \
  --exp-idx 2 \
  --output /path/campaign/sorted/bt_suave_sorted.csv
```

`--done-dir` defaults to the CSV's directory. Supply it when the original
markers live elsewhere.

For multiple systems, create a JSON manifest using actual campaign indices
and container-visible paths:

```json
[
  {"managing_system": "bt", "data_file": "/path/campaign/bt_suave.csv", "exp_idx": 2},
  {"managing_system": "none", "data_file": "/path/campaign/none_suave.csv", "exp_idx": 5}
]
```

Each entry also accepts `done_dir`. Run:

```bash
python3 src/suave/suave_runner/suave_runner/sort_results.py \
  --manifest /path/manifest.json \
  --output /path/campaign/sorted
```

## Interpret and verify

The sorter orders CSV rows by `datetime` and markers by modification time,
then pairs those ordered sequences one to one. It rejects unequal counts,
matched timestamps separated by more than its configured threshold
(currently 120 seconds), and recovered IDs that are not a permutation of
`0..n-1`. It prepends `run_idx`, sorts by it, and preserves the other columns.

Report failures with the affected input and underlying error. Investigate
the CSVs, campaign mapping, and marker provenance instead of trimming rows,
changing marker times, or relaxing the threshold just to obtain a result.

After success, report output paths and recovered run counts. Existing
verified outputs can be reused without sorting again.

Matching `run_idx` values identify shared perturbation configurations only
within a campaign that actually reused those configurations across systems.
Do not use the same numeric ID to pair unrelated campaigns. For subsequent
Wilcoxon analysis, use `$wilcoxon-analysis` or consult the repository's
`suave_runner/WILCOXON_ANALYSIS_SPEC.md` and analysis script directly.
