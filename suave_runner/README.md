# suave_runner

Experiment orchestration and statistical analysis for SUAVE. The package runs
repeatable experiment campaigns and batches of campaigns, diagnoses failed
runs, resumes interrupted campaigns, and analyzes their results.

User documentation lives on the SUAVE documentation site:

- [Run SUAVE](https://kas-lab.github.io/suave/run.html): campaigns, batches,
  configuration, diagnosis, resume, and output layout.
- [Processing results](https://kas-lab.github.io/suave/results.html):
  summaries, Wilcoxon and Mann-Whitney tests, Q-Q plots, and LaTeX tables.
- [SUAVE CLI](https://kas-lab.github.io/suave/cli.html): the `suave run`,
  `suave batch`, `suave campaign`, and `suave analyze` commands that wrap this
  package.

## Quick start

From the root of a SUAVE checkout, with a SUAVE container running:

```bash
suave run                        # campaign from config/runner/runner_config.yml
suave batch start                # campaigns from config/runner/batch_campaigns.yml
suave batch diagnose --latest
suave analyze summarize -- /home/ubuntu-user/suave/results/CAMPAIGN
```

## Contents

### Executables

| Command | Purpose |
|---|---|
| `ros2 run suave_runner suave_runner` | Run one campaign |
| `ros2 run suave_runner run_batch` | Run a sequence of campaigns |
| `ros2 run suave_runner diagnose_batch` | Report failed runs of a batch or campaign (read-only) |
| `ros2 run suave_runner summarize_results` | Descriptive statistics of one campaign |
| `ros2 run suave_runner wilcoxon_analysis` | Paired Wilcoxon tests between managing systems |
| `ros2 run suave_runner mann_whitney_analysis` | Independent-sample Mann-Whitney tests |

Batch analysis scripts run with `python3` from the SUAVE repository root:
`analysis/wilcoxon_analysis_batch.py`, `analysis/qq_plot.py`,
`analysis/qq_plot_batch.py`, and `latex/wilcoxon_latex_tables_batch.py`, all
under `suave_runner/suave_runner/`. Each has a `--help`.

### Launch files

| Launch file | Purpose |
|---|---|
| `suave_runner_launch.py` | Run a campaign from `config/runner/runner_config.yml`, or from `config_file:=PATH` |
| `run_batch_launch.py` | Run the batch in `config/runner/batch_campaigns.yml` |
| `exp1_analysis_launch.py` … `extended_exp3_analysis_launch.py` | Wilcoxon analysis of the bundled campaigns, with `results_root:=CAMPAIGN` |

### Configuration

| File | Contents |
|---|---|
| `config/runner/runner_config.yml` | Default campaign |
| `config/runner/exp{1,2,3}_runner_config.yml` | Original use case campaigns |
| `config/runner/extended_exp{1,2,3}_runner_config.yml` | Extended use case campaigns (`suave_bt_extended`) |
| `config/runner/water_current_runner_config.yml` | Campaign with the water current enabled |
| `config/runner/batch_campaigns.yml` | Batch of the six `exp` and `extended_exp` campaigns |
| `config/analysis/*_analysis_config.yml` | `data_files` mappings for the analysis launch files |

Edit a copy, or pass your own file, for a different campaign. Third-party
managing systems can supply runner configurations from their own packages.

## Package layout

| Module | Responsibility |
|---|---|
| `suave_runner.py` | Campaign runner node: perturbations, readiness checks, attempts |
| `run_batch.py` | Batch runner and its `state.json` manifest |
| `run_status.py` | Run entries of a campaign's `runs.json`, with attempt history |
| `run_artifacts.py` | Attempt bags, campaign lock, and recovery of unfinished attempts |
| `attempt_processes.py` | Launch-session tracking and process shutdown |
| `bag_validation.py` | Validation of finished MCAP recordings |
| `diagnose_batch.py` | Read-only, ROS-independent failure diagnosis |
| `analysis/` | Summaries, statistical tests, Q-Q plots, and shared CSV helpers (`common.py`) |
| `latex/` | Pairwise p-value table renderer and batch LaTeX generation |

Python callers import from `suave_runner.analysis` and `suave_runner.latex`.
The batch scripts import their neighbours, so keep each script in its folder
when invoking it directly.

