# suave_runner

## Docker

Build the headless image from the repository root:

```bash
docker build -t suave-headless:dev -f docker/dockerfile-suave-headless .
```

Run the local image with NVIDIA GPU support:

```bash
docker run -it --rm --gpus all --runtime=nvidia --name suave_runner -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro suave-headless:dev
```

```bash
docker run -it --rm --gpus all --runtime=nvidia --name suave_runner -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro -v $HOME/ros_workspaces/suave_rebetmc_ws/src/suave:/home/ubuntu-user/suave_ws/src/suave suave-headless:dev
```

The published image is available as `ghcr.io/kas-lab/suave-headless:main`.

```Bash
docker run -it --rm --gpus all --runtime=nvidia --name suave_runner -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -v $HOME/suave/results:/home/ubuntu-user/suave/results -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro ghcr.io/kas-lab/suave-headless:main
```

## Resuming a crashed campaign

If the runner crashes or is interrupted mid-campaign, it can resume from where it
left off without re-running completed runs.

After each successful run the runner writes a marker file
`run_<exp_idx>_<run_idx>.done` inside the result folder. On resume the runner
reads these markers and skips any run whose marker already exists. Runs that
timed out (no `mission_metrics/done` received) are **not** marked and will be
retried.

The runner also ends a run immediately when a launched process exits with a
nonzero status or a subsystem publishes `mission/control_failure`. Failed runs
are left without a marker, and the runner continues with the remaining runs.
Process stdout, stderr, launch events, and ROS logs are retained below
`logs/run_<exp_idx>_<run_idx>/`; setting `experiment_logging` to `false` only
suppresses console output.

To resume, pass the path of the existing result folder via `resume_result_path`:

```bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p resume_result_path:=~/suave/results/2026_06_19_10-30-00 \
  -p experiments:='[...]'
```

The `experiments` parameter must match the original campaign. Per-run mission
config files (`mission_config_run*.yaml`) are reused if already present in the
folder; otherwise they are regenerated from the same random seed.

## Reproducible perturbations

The runner uses the `random_seed` ROS parameter when generating experiment
perturbations. It defaults to `100` so benchmark runs are reproducible. Set a
different integer in `config/runner_config.yml` or through `--ros-args -p
random_seed:=<value>` to generate a different sequence.


## Metacontrol

```bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p gui:=True \
  -p experiments:='[
    "{\"experiment_launch\": \"ros2 launch suave_bringup mission.launch.py adaptation_manager:=metacontrol\", \
      \"num_runs\": 1, \
      \"adaptation_manager\": \"metacontrol\", \
      \"mission_name\": \"suave\"}"
  ]'
```

## Behavior Tree

Append `use_action_server:=true` to `experiment_launch` to have the BT invoke
managed behaviors through ROS 2 actions:

Without action server
```Bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p gui:=False \
  -p experiments:='[
    "{\"experiment_launch\": \"ros2 launch suave_bringup mission.launch.py adaptation_manager:=bt\", \
      \"num_runs\": 1, \
      \"adaptation_manager\": \"bt\", \
      \"mission_name\": \"suave\"}"
  ]'
```

With action server:
```Bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p gui:=False \
  -p experiments:='[
    "{\"experiment_launch\": \"ros2 launch suave_bringup mission.launch.py adaptation_manager:=bt use_action_server:=true\", \
      \"num_runs\": 20, \
      \"adaptation_manager\": \"bt\", \
      \"mission_name\": \"suave\"}"
  ]'
```

## Random

```bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p gui:=False \
  -p experiment_logging:=True \
  -p experiments:='[
    "{\"experiment_launch\": \"ros2 launch suave_bringup mission.launch.py adaptation_manager:=random\", \
      \"num_runs\": 2, \
      \"adaptation_manager\": \"random\", \
      \"mission_name\": \"suave\"}"
  ]'
```

## None

```bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p gui:=False \
  -p experiment_logging:=True \
  -p experiments:='[
    "{\"experiment_launch\": \"ros2 launch suave_bringup mission.launch.py adaptation_manager:=none\", \
      \"num_runs\": 6, \
      \"adaptation_manager\": \"none\", \
      \"mission_name\": \"suave\"}"
  ]'
```

## Mann-Whitney analysis

The `mann_whitney_analysis` executable
compares mission-level metrics produced by two or more managing systems,
treating each system's runs as independent samples. For each ordered pair of
systems, it reports Shapiro-Wilk normality diagnostics and performs one-sided
Mann-Whitney U tests for the following alternatives:

| Metric | Alternative for row system A against column system B |
|---|---|
| Time searching for the pipeline | A is lower than B |
| Distance inspected | A is greater than B |
| Mean reaction time | A is lower than B |

Each input CSV file must contain these columns exactly:

- `time searching pipeline (s)`
- `distance inspected (m)`
- `mean reaction time (s)`

These columns are written by `suave_metrics`. If a run does not find the
pipeline, its search time equals its full mission duration. If a run has no
adaptation events, its mean reaction time is `0.0`.

Run the analysis from a sourced SUAVE workspace:

```bash
ros2 run suave_runner mann_whitney_analysis \
  --ros-args \
  -p result_path:=~/suave/results/mann_whitney_analysis \
  -p filename:=none_vs_bt \
  -p data_files:='[
    "{\"managing_system\": \"none\", \
      \"data_file\": \"~/suave/results/none_suave.csv\"}",
    "{\"managing_system\": \"suave_bt\", \
      \"data_file\": \"~/suave/results/bt_suave.csv\"}"
  ]'
```

The ROS parameters are:

| Parameter | Default | Description |
|---|---|---|
| `data_files` | `['']` (placeholder) | Required array of JSON strings containing `managing_system` and `data_file` |
| `result_path` | `~/suave/results` | Directory in which result matrices are written |
| `filename` | `suave_statistical_analysis` | Prefix used for result matrix filenames |

The command prints normality p-values and directional test conclusions using a
significance threshold of `0.05`. The normality results are diagnostic only and
do not alter the Mann-Whitney tests. The command also writes three CSV matrices:

- `<filename>_time_search_pipeline.csv`
- `<filename>_distance_inspected.csv`
- `<filename>_mean_reaction_time.csv`

Matrix rows and columns are managing-system names. Cell `(A, B)` contains the
raw p-value for the directional alternative shown above, and diagonal cells are
empty. The script does not apply a multiple-comparison correction. A p-value at
or above `0.05` means that the analysis did not find sufficient evidence for
the specified direction; it does not establish equality or the opposite
direction. Mann-Whitney U compares sample ranks and distributions rather than
arithmetic means directly.

Mann-Whitney treats each system's runs as unrelated samples. If the campaign
reused the same perturbation configuration across managing systems (SUAVE
campaigns do, see `sort-suave-results`), use the paired Wilcoxon analysis
below instead -- it controls for per-scenario difficulty and is more
statistically powerful, but requires a run-identity column that a resumed
campaign's raw CSVs do not carry by default.

## Wilcoxon analysis (paired)

The `wilcoxon_analysis` executable implements a paired Wilcoxon
signed-rank test on matched runs across managing systems, instead of treating
them as independent samples. See the `wilcoxon-analysis` skill for the full
workflow; summary:

**Prerequisite: sort first.** Every input CSV must carry a `run_idx` column
identifying which run of that managing system each row is. A `--resume`d
campaign appends retried runs out of order, so this cannot be assumed from
row position. Recover it first with `sort_results.py` (see the
`sort-suave-results` skill):

```bash
python3 suave_runner/analysis/sort_results.py \
  --csv ~/suave/results/bt_suave.csv --exp-idx 2 \
  --output ~/suave/results/sorted/bt_suave_sorted.csv
```

Then run the paired analysis on the sorted CSVs:

```bash
ros2 run suave_runner wilcoxon_analysis \
  --ros-args \
  -p result_path:=~/suave/results/wilcoxon_analysis \
  -p filename:=none_vs_bt \
  -p correction:=holm \
  -p data_files:='[
    "{\"managing_system\": \"none\", \
      \"data_file\": \"~/suave/results/sorted/none_suave_sorted.csv\"}",
    "{\"managing_system\": \"bt\", \
      \"data_file\": \"~/suave/results/sorted/bt_suave_sorted.csv\"}"
  ]'
```

Parameters beyond those shared with `mann_whitney_analysis`:

| Parameter | Default | Description |
|---|---|---|
| `correction` | `holm` | Multiple-comparison correction across every pair and metric in the run; set to any other value to disable (raw p-values are then copied into `p_adjusted`) |

Unlike `mann_whitney_analysis`, `data_files` here must point at
`run_idx`-tagged, sorted CSVs. A missing `run_idx` column, a duplicate
`run_idx`, or an unmatched run raises rather than falling back to row order.

The command writes:

- `<filename>_results.csv` -- the primary output, one row per ordered system
  pair and metric (`search_time` and `distance_inspected`), with the
  alternative hypothesis, pair counts
  (declared / used / positive / negative / zero), median paired difference,
  test statistic, raw and Holm-adjusted p-values, a matched rank-biserial
  effect size, and the run indices excluded at each filtering step (missing
  counterpart, pipeline not found for either metric);
- `<filename>_time_search_pipeline.csv` and
  `<filename>_distance_inspected.csv` -- secondary compatibility matrices in
  the same shape `mann_whitney_analysis` writes (raw p-values, not
  Holm-adjusted), so existing table-generation tooling can point at either.

Both `search_time` and `distance_inspected` exclude any pair where either
system did not find the pipeline. Search time is right-censored at the
mission duration in that case, and inspection requires finding the pipeline.
Both comparisons are therefore conditional on detection by both systems.
Holm correction applies across the ordered system pairs for these two metrics.

## Result summary

The `summarize_results` executable produces descriptive statistics for all
managing systems in one campaign folder. It discovers aggregate result files
named `<managing_system>_suave.csv`, such as `bt_suave.csv` and
`metacontrol_suave.csv`. Other CSV files, including per-event reaction-time and
reasoning-time files, are ignored.

Each aggregate result file must contain at least these columns:

- `pipeline found`;
- `time searching pipeline (s)`;
- `distance inspected (m)`; and
- `mean reaction time (s)`.

The `pipeline found` column accepts `true`/`false`, `1`/`0`, or `yes`/`no`
(case-insensitive). Metric columns must contain numeric, non-missing values,
and each file must contain at least one result row.

Run the command from a sourced SUAVE workspace, passing the campaign folder as
the positional argument:

```bash
ros2 run suave_runner summarize_results \
  ~/suave/results/2026_08_01_19-42-44
```

The terminal table reports the following information for each managing system:

- pipeline detection success as both `successful/total` and a percentage;
- mean and sample standard deviation of the pipeline search time;
- mean and sample standard deviation of the distance inspected; and
- mean and sample standard deviation of the reaction time.

Search time and distance use every run. Reaction-time calculations exclude
zero-valued runs because zero indicates that no reaction-time sample was
recorded. Reaction time is reported as `N/A` if no nonzero samples remain. The
sample standard deviation is also `N/A` when fewer than two applicable samples
are available.

Managing-system identifiers are derived from filenames and formatted for
presentation. The known identifiers are `bt`, `metacontrol`, `none`, `random`,
and `rebetmc`, displayed as `BT`, `Metacontrol`, `None`, `Random`, and
`ReBeT-MC`, respectively. Other identifiers are converted from snake case to
title case.

Use `--latex` to also write a complete LaTeX table to
`<results_path>/suave_results.tex`:

```bash
ros2 run suave_runner summarize_results \
  ~/suave/results/2026_08_01_19-42-44 \
  --latex
```

Use `--latex-output` to select another output file. Parent directories are
created automatically. Supplying this option also enables LaTeX generation,
so `--latex` is not required:

```bash
ros2 run suave_runner summarize_results \
  ~/suave/results/2026_08_01_19-42-44 \
  --latex-output ~/paper/tables/suave_results.tex
```

The available arguments are:

| Argument | Default | Description |
|---|---|---|
| `results_path` | Required | Campaign folder containing the aggregate `*_suave.csv` files |
| `--latex` | Disabled | Write `<results_path>/suave_results.tex` |
| `--latex-output PATH` | Not set | Write the table to `PATH`; also enables LaTeX output |
| `--precision N` | `2` | Use `N` decimal places in terminal and LaTeX output |

The LaTeX output is a complete `table` environment with the caption
`SUAVE results mean and standard deviation.` and label `tab:suave_results`.
It uses `\toprule`, `\midrule`, and `\bottomrule` from `booktabs` and
`\resizebox` from `graphicx`, so include both packages in the parent document:

```latex
\usepackage{booktabs}
\usepackage{graphicx}
```

The generated file can then be included with `\input`, for example:

```latex
\input{tables/suave_results}
```


## Batch paired Wilcoxon analysis

`wilcoxon_analysis_batch.py` orchestrates the existing `wilcoxon_analysis.py`
node. Run it from the sourced SUAVE workspace root inside the container:

```bash
python3 src/suave/suave_runner/suave_runner/analysis/wilcoxon_analysis_batch.py \
  /home/ubuntu-user/suave/results/batches/all_experiments_20260904_093305
```

It first sorts the raw aggregate CSVs in each `campaigns/<experiment>/`
using the original `run_<exp_idx>_<run_idx>.done` markers, then analyzes
the verified copies in `sorted/` and writes:

```text
campaings_results/
  exp1/
    q-q-plots/
    wilcoxon_analysis/
      exp1_wilcoxon_results.csv
      exp1_wilcoxon_time_search_pipeline.csv
      exp1_wilcoxon_distance_inspected.csv
  exp2/
    wilcoxon_analysis/
      ...
```

`--output PATH` overrides `campaings_results`, preserving the
`<experiment>/wilcoxon_analysis/` structure. Keep this script alongside
`wilcoxon_analysis.py` and `sort_results.py`. No package rebuild is needed
for this direct invocation.

Each campaign's runner configuration is resolved from `state.json`, supporting
both the original batch launch-name keys and the newer campaign-name keys.
The configuration's `experiments` list determines the method-to-`exp_idx`
mapping, filenames, and expected run counts. If recorded config paths have
moved, pass `--config-dir /path/to/original/configs`; this directory must
contain the recorded config basenames (or `<experiment>_runner_config.yml`
when no config is recorded). Use configs that match the original campaign.
The script does not infer marker indices from alphabetical method order.

The pre-step calls the existing `sort_results.reconstruct_run_index` without
relaxing its count, timestamp-gap, or run-ID checks. Original CSVs and marker
modification times are preserved. Run in the timezone used when the CSV
`datetime` values were recorded. All configured methods are reconstructed and
validated before any sorted CSV is updated; unchanged verified sorted files
retain their modification times, and stale ones are refreshed. Only methods
listed in that campaign configuration are analyzed, so unrelated old files
in `sorted/` are ignored. Missing markers, configs, or raw CSVs, timestamp
mismatches, and incomplete configured run counts fail that campaign before
analysis instead of falling back to possibly stale sorted data.

Runs must share experimental configurations within each campaign; never
match unrelated campaigns just because their numeric IDs agree. The
batch wrapper rejects missing or duplicate IDs, noninteger or negative IDs,
unmatched run-key sets, missing/nonfinite/nonnumeric metric values, and
invalid or missing `pipeline found` status before testing that campaign.
It reports failed campaigns, continues the others, and returns a nonzero
exit status if any campaign fails.

The existing statistical design is preserved: all ordered method pairs are
tested with A minus B, using `less` for search time and `greater` for distance
inspected. Both metrics exclude a pair if either method failed to find the
pipeline. Zero distance is retained when both methods found it. This differs
from the Q-Q script, which includes every complete numeric pair.

Holm correction pools all computed ordered-pair tests across both metrics
**within each campaign**, separately from the other campaigns. The default is
`--correction holm`; `--correction none` explicitly disables adjustment.

The primary `*_results.csv` reports raw and adjusted p-values, paired counts,
positive/negative/zero differences, median difference, rank-biserial effect
size, exclusions, and notes. Fewer than two nonzero differences produces an
uncomputed test with a note and missing p-values. `n_used` includes zeros;
`n_pos + n_neg` is the effective nonzero count. Negative effects favor A for
search time; positive effects favor A for distance. The secondary matrices
and the existing engine's console significance labels use **raw** p-values;
use `p_adjusted` in the primary CSV when interpreting corrected results.
A nonsignificant result does not establish equality. Independent pairs and
reasonably symmetric paired differences are needed for the usual
location-shift interpretation.
