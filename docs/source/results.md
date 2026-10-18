# Processing results

This page describes campaign summaries, statistical comparisons, Q-Q plots, and
LaTeX tables. See {doc}`run` for campaign execution, output files, failure
diagnosis, and recovery.

## Choose a tool

| Task | Tool | Experimental design |
|---|---|---|
| Summarize detection rates and mission metrics | [Result summary](#result-summary) | One campaign |
| Compare systems on matched scenarios | [Wilcoxon analysis](#wilcoxon-analysis-paired) | Paired runs |
| Compare independent samples | [Mann-Whitney analysis](#mann-whitney-analysis) | Independent runs |
| Inspect paired-difference distributions | [Q-Q plots](#q-q-plots-of-paired-differences) | Paired observations |
| Generate pairwise comparison tables | [Wilcoxon LaTeX tables](#wilcoxon-latex-tables) | Existing Wilcoxon results |

Use the SUAVE CLI ({doc}`cli`) for summaries and single-campaign statistical
comparisons. Pass analysis arguments after `--`. The CLI does not wrap the
batch analysis, Q-Q plotting, or pairwise LaTeX scripts.

## Inputs and filtering

A campaign folder contains raw `<manager>_<mission>.csv` files and `runs.json`.
The tools differ in how they select observations.

| Tool | Attempt selection | Metric filtering |
|---|---|---|
| Summary | All CSV rows, including rows from failed attempts | Search time and distance require pipeline detection. Reaction time excludes zero values |
| Mann-Whitney | All CSV rows, including rows from failed attempts | Uses the supplied metric samples without the paired-detection filter |
| Wilcoxon | Completed attempts identified by `runs.json`, `run_idx`, and `attempt_id` | Both systems must detect the pipeline for both metrics |
| Q-Q, separate campaign CSVs | Completed attempts identified by `runs.json`, `run_idx`, and `attempt_id` | Keeps complete numeric pairs, including zeros and nondetections |

For paired analyses, verify that matching run IDs represent the same scenario
and compatible non-treatment settings. Equal IDs alone do not establish pairing
across campaigns. Batch tools compare systems within each campaign, never across
campaigns.

The completed-attempt loader rejects missing identity columns, missing
`runs.json`, managers without completed runs, duplicate accepted rows, and
completed runs without CSV rows. These checks do not establish scenario equivalence.

The metric column names are:

- `pipeline found`
- `time searching pipeline (s)`
- `distance inspected (m)`
- `mean reaction time (s)`

The summary requires all four columns. Wilcoxon requires the first three.
Mann-Whitney requires the three numeric metrics. Q-Q plots use the selected
numeric columns. Detection values accept `true`/`false`, `1`/`0`, or `yes`/`no`,
without case sensitivity. Validate metric values before analysis.

## Result summary

`summarize_results` discovers `<manager>_suave.csv` files in one campaign folder
and ignores per-event CSVs. Each input file must contain at least one row and
numeric, non-missing metric values.

Use the CLI to print statistics and optionally export a LaTeX table:

```bash
suave analyze summarize -- \
  /home/ubuntu-user/suave/results/batches/BATCH/campaigns/exp3 \
  --latex-output /tmp/suave_results.tex
```

The direct ROS equivalent is `ros2 run suave_runner summarize_results`, followed
by the same path and options.

| Argument | Default | Description |
|---|---|---|
| `results_path` | Required | Campaign folder containing `*_suave.csv` files |
| `--latex` | Disabled | Write `<results_path>/suave_results.tex` |
| `--latex-output PATH` | Not set | Write to `PATH` and create parent directories. Also enables LaTeX output |
| `--precision N` | `2` | Decimal places in terminal and LaTeX output |

The summary reports detection counts and percentages, and the mean and sample
standard deviation of each metric. It applies the filters in
[Inputs and filtering](#inputs-and-filtering). It reports `N/A` when no applicable
observations remain, or when fewer than two observations support a sample
standard deviation. Zero reaction time indicates that the run recorded no
reaction-time sample.

The tool derives manager identifiers from filenames and uses the display labels
in `suave_tools/labels.py`. Unknown identifiers remain unchanged.

The LaTeX output contains a complete `table` environment with label
`tab:suave_results`. Include it in a document with:

```latex
\usepackage{booktabs,graphicx}
% In the document body:
\input{tables/suave_results.tex}
```

## Wilcoxon analysis (paired)

`wilcoxon_analysis` applies one-sided Wilcoxon signed-rank tests to every ordered
pair of systems. Use it for matched scenarios, as in SUAVE campaigns that reuse
perturbation configurations across systems.

### Single-campaign usage

Run the CLI with a ROS parameter file:

```bash
suave analyze wilcoxon -- --ros-args --params-file my_analysis.yml
```

Example `my_analysis.yml`:

```yaml
/wilcoxon_analysis:
  ros__parameters:
    result_path: /home/ubuntu-user/suave/results/campaign/wilcoxon_analysis
    filename: none_vs_bt
    correction: holm
    data_files:
      - '{"managing_system": "none", "data_file": "/home/ubuntu-user/suave/results/campaign/none_suave.csv"}'
      - '{"managing_system": "bt", "data_file": "/home/ubuntu-user/suave/results/campaign/bt_suave.csv"}'
```

Replace the paths with raw CSVs from the same campaign. Use unique system labels.
The direct ROS equivalent is:

```bash
ros2 run suave_runner wilcoxon_analysis --ros-args --params-file my_analysis.yml
```

For the bundled `exp1` through `exp3` and `extended_exp1` through `extended_exp3`
campaigns, preconfigured launch files provide the CSV mappings:

```bash
ros2 launch suave_runner exp1_analysis_launch.py \
  results_root:=/home/ubuntu-user/suave/results/campaign correction:=holm
```

`results_root` identifies the campaign folder. `output_root` defaults to
`results_root`. Check the mappings in
[exp1_analysis_config.yml](https://github.com/kas-lab/suave/blob/main/suave_runner/config/analysis/exp1_analysis_config.yml)
when the campaign uses different systems.

### Parameters

Wilcoxon and Mann-Whitney share the following ROS parameters, with different
filename defaults:

| Parameter | Default | Description |
|---|---|---|
| `data_files` | `['']` | Required array of JSON strings containing `managing_system` and `data_file` |
| `result_path` | `~/suave/results` | Output directory |
| `filename` | `suave_wilcoxon_analysis` for Wilcoxon, `suave_statistical_analysis` for Mann-Whitney | Output filename prefix |
| `correction` | `none` | Wilcoxon only. Set `holm` for Holm-Bonferroni correction |

### Batch usage

The batch script analyzes every manager listed in each campaign's `runs.json`,
in experiment order. Each campaign needs at least two managers.

```bash
python3 src/suave/suave_runner/suave_runner/analysis/wilcoxon_analysis_batch.py \
  /home/ubuntu-user/suave/results/batches/BATCH --correction holm
```

The batch scripts accept a batch root, its `campaigns` directory, its `state.json`,
or one campaign folder. Wilcoxon writes to
`<batch>/campaign_results/<experiment>/wilcoxon_analysis/` by default.
`--output PATH` replaces the `campaign_results` root. `--correction` defaults to
`none`.

The batch wrapper requires identical run-ID sets, nonnegative integer IDs,
finite numeric metrics, and valid detection values. The single-campaign engine
instead uses the intersection of run IDs and reports unmatched IDs in its
output. Check and resolve incomplete pairing before interpreting that output.
The engine does not provide the batch wrapper's full input validation.

### Statistical rules and outputs

For systems A and B, the paired difference is A minus B:

| Metric | Alternative | Effect direction favoring A |
|---|---|---|
| Search time | A is lower than B (`less`) | Negative |
| Distance inspected | A is greater than B (`greater`) | Positive |

Both metrics require detection by both systems. Nondetection censors search
time at the mission duration and prevents inspection. Retain zero distance
when both systems found the pipeline. The comparisons therefore describe
performance conditional on detection, not overall detection success.

Holm correction pools all computed ordered-pair tests across both metrics in
one invocation. The batch script applies correction separately within each
campaign. With `correction: none`, `p_adjusted` equals `p_raw`.

| Output | Contents |
|---|---|
| `<filename>_results.csv` | Primary output with pair counts, positive/negative/zero differences, median difference, test statistic, raw and adjusted p-values, rank-biserial effect size, exclusions, and notes |
| `<filename>_time_search_pipeline.csv` | Search-time matrix of raw p-values |
| `<filename>_distance_inspected.csv` | Distance matrix of raw p-values |

Batch filenames use `<experiment>_wilcoxon` as the prefix. Matrix cell `(A, B)`
tests the directional alternative for A against B. Diagonal cells are empty.

The engine does not compute a test with fewer than two nonzero differences.
`n_used` includes zeros, while `n_pos + n_neg` counts nonzero differences.
Use `p_adjusted` from the primary CSV for corrected conclusions. Secondary
matrices and console significance labels use raw p-values.

A nonsignificant result does not establish equality. The usual location-shift
interpretation requires independent pairs and reasonably symmetric paired
differences. Normality of individual system samples does not determine whether
the experimental design is paired.

## Mann-Whitney analysis

`mann_whitney_analysis` treats systems' runs as independent samples. Use it when
the experimental design does not match scenarios across systems. It compares
sample ranks and distributions, not arithmetic means directly.

```bash
suave analyze mann-whitney -- --ros-args --params-file independent_analysis.yml
```

Use the [shared parameters](#parameters) in a ROS parameter file under
`/analysis`, the Mann-Whitney node name. Omit `correction` and supply CSVs
from independent samples. The direct ROS equivalent is `ros2 run suave_runner
mann_whitney_analysis`, followed by the same ROS arguments.

The tool reports Shapiro-Wilk normality diagnostics and one-sided Mann-Whitney
U tests. The diagnostics do not alter the tests. Alternatives are lower search
time, greater distance inspected, and lower mean reaction time for row system A
against column system B.

The tool writes raw-p-value matrices with suffixes `_time_search_pipeline.csv`,
`_distance_inspected.csv`, and `_mean_reaction_time.csv`. Diagonal cells are
empty. It uses a significance threshold of `0.05` and applies no
multiple-comparison correction. A nonsignificant result does not establish
equality or the opposite direction.

Unlike Wilcoxon, this tool includes nondetections and failed-attempt CSV rows.
Nondetection sets search time to the mission duration. A run without adaptation
events has mean reaction time `0.0`.

## Q-Q plots of paired differences

`qq_plot.py` plots A minus B against normal quantiles to inspect the paired
t-test normality assumption. It fits a reference line to the differences but
does not perform a formal normality test or classify the distribution.

### Single comparisons

Compare two columns in one CSV, paired by row:

```bash
python3 src/suave/suave_runner/suave_runner/analysis/qq_plot.py results.csv \
  --column-a suave --column-b baseline --output qq_suave_baseline.png
```

Compare one metric in two raw CSVs from the same campaign:

```bash
python3 src/suave/suave_runner/suave_runner/analysis/qq_plot.py \
  /path/to/campaign/metacontrol_suave.csv /path/to/campaign/bt_suave.csv \
  --column-a 'time searching pipeline (s)' \
  --column-b 'time searching pipeline (s)' --output qq_search.png
```

Separate files use completed attempts and match unique `run_idx` values,
regardless of row order. `--run-column` selects another pairing key.
The tool rejects unmatched or duplicate IDs. `--label-a` and `--label-b`
override filename-derived labels. Without `--output`, the tool displays the
plot interactively.

### Batch plots

```bash
python3 src/suave/suave_runner/suave_runner/analysis/qq_plot_batch.py \
  /home/ubuntu-user/suave/results/batches/BATCH
```

The script accepts the [batch input forms](#batch-usage). It reads managers
from `runs.json` and plots every unordered pair for search time and distance
inspected by default. It always saves figures without a display.

| Option | Default | Description |
|---|---|---|
| `--metrics COLUMN ...` | Search time and distance inspected | Metric columns to plot |
| `--run-column COLUMN` | `run_idx` | Pairing key |
| `--output PATH` | `<batch>/campaign_results` | Output root with `<experiment>/q-q-plots/` subdirectories |
| `--format pdf\|png\|svg` | `png` | Figure format |

Both scripts remove a pair if either value is missing, reject infinite or
nonnumeric values, and require at least three complete pairs. They retain
zero metrics and nondetections, unlike Wilcoxon. They print the retained and
removed pair counts, mean, median, sample standard deviation, minimum, and
maximum of the differences.

## Wilcoxon LaTeX tables

Run the batch Wilcoxon analysis first, then generate one table per campaign:

```bash
python3 src/suave/suave_runner/suave_runner/latex/wilcoxon_latex_tables_batch.py \
  /home/ubuntu-user/suave/results/batches/BATCH
```

The script accepts the [batch input forms](#batch-usage) and reads
`<batch>/campaign_results/<experiment>/wilcoxon_analysis/*_results.csv`.
It uses `p_adjusted` by default and does not recompute tests or corrections.
Each table contains search-time and distance-inspected blocks.

| Option | Default | Description |
|---|---|---|
| `--results-root PATH` | `<batch>/campaign_results` | Analysis input root |
| `--output PATH` | `<batch>/campaigns_latex_tables/wilcoxon_analysis` | Directory for all generated tables |
| `--p-values adjusted\|raw` | `adjusted` | P-value column to display |
| `--alpha VALUE` | `0.05` | Cell-color threshold |

Captions identify the p-value policy, alternatives, and detection condition.
Green cells indicate p-values below alpha. Blue cells indicate other computed
values. Colors use unrounded values. Printed values use three decimal places,
with values that would round to `0.000` shown as `<0.001`. A dash indicates a
diagonal cell or an uncomputed test. Consult the primary CSV for exclusions and
uncomputed-test notes.

The renderer validates complete, unique ordered pairs, alternatives, correction
policy, p-values, and missing-result notes. Labels and ordering follow
`suave_tools/labels.py`. Unknown managers retain their raw keys. Tables use
`0.7\textwidth` for up to three systems and `\textwidth` otherwise. Rerunning
the command refreshes the generated files.

Include a generated table with:

```latex
\usepackage{array,graphicx,multirow}
\usepackage[table]{xcolor}
\definecolor{pvalue_green}{RGB}{210,245,210}
\definecolor{pvalue_blue}{RGB}{210,225,245}
% In the document body:
\input{path/to/exp1_wilcoxon.tex}
```

## Output layout and tool locations

The batch commands create separate outputs. No single command creates the
entire structure below:

```text
BATCH/
  campaigns/
    exp1/runs.json, *.csv
  campaign_results/
    exp1/
      q-q-plots/*.png
      wilcoxon_analysis/
        exp1_wilcoxon_results.csv
        exp1_wilcoxon_time_search_pipeline.csv
        exp1_wilcoxon_distance_inspected.csv
  campaigns_latex_tables/
    wilcoxon_analysis/
      exp1_wilcoxon.tex
```

Batch tools report failures, continue with remaining campaigns or comparisons,
and return a nonzero exit status if any operation fails.

The Python tools reside under `suave_runner/suave_runner/`:

| Directory | Tools |
|---|---|
| `analysis/` | Summaries, statistical tests, Q-Q plots, batch analysis, and shared CSV helpers |
| `latex/` | Pairwise table renderer and batch LaTeX generation |

Keep this package structure when invoking scripts directly. Use each script's
`--help` for its command-line reference.
