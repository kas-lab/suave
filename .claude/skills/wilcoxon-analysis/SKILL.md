---
name: wilcoxon-analysis
description: Compare SUAVE managing systems on matched campaign runs using Wilcoxon signed-rank tests and Holm correction for search time and distance inspected. Use when systems share verified scenario configurations; independent campaigns require a different design.
---

# SUAVE Wilcoxon Analysis

Use `suave_runner/suave_runner/wilcoxon_analysis.py` for paired comparisons
of search time and distance inspected. Reaction-time and McNemar analyses
are outside this executable's scope.

## Repository and execution

The SUAVE checkout is normally
`~/ros_workspaces/suave_ws/src/suave`, but do not assume this is the correct
one, there might be multiple working versions of SUAVE in the same setup.
Locate the corresponding SUAVE checkout when working elsewhere. Read the
statistical design and current behavior straight from
`suave_runner/suave_runner/analysis/wilcoxon_analysis.py`'s module docstring
and `METRICS` table -- that implementation is the ground truth.

Follow that checkout's `AGENTS.md`: run SUAVE analysis inside its applicable
development or integration container with the default sourced workspace.
Use paths visible inside the container. Rebuild `suave_runner` if needed to
make an updated executable available through `ros2 run`.

## Establish pairing before analysis

- Confirm that the selected CSVs come from a campaign with shared
  perturbation configurations and compatible non-treatment settings.
  Numeric `run_idx` values alone do not establish comparability across
  separate campaigns. Use an independent-samples analysis such as
  `mann_whitney_analysis` when the experimental design is independent.
- Every CSV needs verified, non-missing, unique `run_idx` values. If these
  are absent or their provenance is uncertain, use the `sort-suave-results` skill
  or the repository's `sort_results.py` with the original completion
  markers. Reuse already verified CSVs. Never pair by row position or
  initial position alone.
- Check that the selected systems have matching run-key sets. The current
  implementation reports unmatched keys but silently uses an inner join;
  do not claim it rejects mismatches. Report and resolve incomplete pairing
  before interpreting a result, unless the user explicitly selects an
  incomplete-pair policy.
- Require finite numeric `time searching pipeline (s)` and
  `distance inspected (m)` values and valid `pipeline found` status for
  every input row. The current loader recognizes case-insensitive
  `true`/`false`; missing status can bypass censor filtering, and missing
  numeric values are not filtered reliably. Report invalid inputs before
  testing rather than silently imputing values.

## Run the analysis

For `exp1`-`exp3` and `extended_exp1`-`extended_exp3` (the campaigns bundled
with this repo's own runner configs), prefer the matching launch file under
`suave_runner/launch/analysis/`, e.g.:

```bash
ros2 launch suave_runner exp1_analysis_launch.py \
  results_root:=/path/campaign/sorted
```

`results_root` must point at the directory holding that campaign's
run_idx-sorted CSVs (e.g. `sort_results.py`'s `--output` directory, or a
batch's `campaigns/exp1/sorted/`). `output_root` (defaults to
`results_root`) and `correction` (defaults to `none`) can also be overridden.
See `suave_runner/config/analysis/exp1_analysis_config.yml` for the exact
`data_files` mapping used -- add or edit an entry there (or in a copy) for a
campaign whose managing systems differ from what's bundled.

For any other combination of systems or campaigns, invoke the node directly:

```bash
ros2 run suave_runner wilcoxon_analysis \
  --ros-args \
  -p result_path:=/path/campaign/wilcoxon_analysis \
  -p filename:=campaign_wilcoxon \
  -p correction:=none \
  -p data_files:='["{\"managing_system\": \"none\", \"data_file\": \"/path/campaign/sorted/none_suave_sorted.csv\"}", "{\"managing_system\": \"bt\", \"data_file\": \"/path/campaign/sorted/bt_suave_sorted.csv\"}"]'
```

`data_files` is a ROS string array whose elements are JSON objects with
`managing_system` and `data_file`. Use unique system labels. `correction`
defaults to `none`; set it to `holm` explicitly to enable Holm-Bonferroni
correction.

## Computation and reporting

For row system A versus column system B, the paired difference is `A - B`:

| Result metric | CSV column | Alternative | Pair filtering |
|---|---|---|---|
| `search_time` | `time searching pipeline (s)` | `less` | Exclude a pair if either system did not find the pipeline |
| `distance_inspected` | `distance inspected (m)` | `greater` | Exclude a pair if either system did not find the pipeline |

Both metrics are conditional on both systems finding the pipeline. Apply
this exclusion to all systems, including `random`: failed detection censors
search time and prevents pipeline inspection. Retain zero inspected
distances when both systems found the pipeline. Describe this condition
when reporting results. Holm correction pools all computed ordered-pair
hypotheses across these two metrics in the invocation.

Read `<filename>_results.csv` as the primary output. It reports matched and
used pair counts, positive/negative/zero differences, median paired
difference, Wilcoxon statistic, `p_raw`, `p_adjusted`, correction policy,
matched rank-biserial effect size, exclusions, and notes. With fewer than
two nonzero differences, the test is not computed. `n_used` includes zero
differences; report `n_pos + n_neg` as the effective nonzero count.

The secondary `<filename>_time_search_pipeline.csv` and
`<filename>_distance_inspected.csv` matrices contain raw p-values. The
current console significance labels also use raw p-values. Base corrected
significance claims on `p_adjusted` in the tidy CSV when Holm is enabled.

Report the alternatives, correction family, counts, effect direction,
exclusions, and any uncomputed tests. A nonsignificant result does not
establish equality. The usual location-shift interpretation requires
independent pairs and reasonably symmetric paired differences; normality
of raw samples does not determine whether the design is paired.

When manuscript updates are requested, check whether the resulting claims
change and update the surrounding prose along with the tables. Do not
modify the manuscript merely because an analysis was requested.
