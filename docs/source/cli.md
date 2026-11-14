# SUAVE CLI

The `suave` command wraps Docker, colcon, and ROS commands for development and
experiment execution. It calls the existing scripts and launch files rather
than replacing them. This page documents CLI behavior. See {doc}`run` for
campaign configuration and recovery, {doc}`results` for analysis, and
{doc}`docker` for image variants and direct Docker commands.

## Setup and requirements

The CLI requires Python 3.10 or newer and uses only the Python standard library.
It does not import ROS. A host with Docker can therefore invoke ROS commands
inside a SUAVE container without installing ROS locally.

**A container is optional.** Build, test, campaign, batch, and analysis commands
can also run directly on the host when ROS 2 Humble, the SUAVE workspace, and
the dependencies required by the selected command are installed. See
{doc}`installation` for local setup.

From the checkout root, add the CLI to the current Bash session:

```bash
source env.sh
suave --help
suave config show
```

Alternatively, build the optional `suave_cli` package and source the workspace,
or symlink `<checkout>/suave_cli/bin/suave` into a directory on `PATH`. The SUAVE
images already provide the command. Without changing `PATH`, invoke
`suave_cli/bin/suave` from the checkout root.

The CLI uses the checkout selected by `--suave-root`, then `SUAVE_ROOT`, then the
location of its launcher. Configuration files belong to that checkout.

## Quick start

On a host with Docker:

```bash
suave docker run --gpu none
suave docker status
suave test suave_runner
suave --dry-run run
suave docker shell
```

`docker run` selects an available SUAVE image or offers to pull one. Omit
`--gpu none` to request NVIDIA GPU access, which requires the NVIDIA Container
Toolkit. `test` builds the selected package before testing it. `--dry-run run`
prints the campaign command without starting the campaign. `docker shell`
opens a shell with ROS and the workspace sourced.

To build an image from the checkout instead of pulling one, use
`suave docker build` before `docker run`. See {doc}`run` before executing
`suave run`, which uses the installed multi-run campaign configuration.

## Where commands run

Docker-management commands run on the host. ROS commands (`build`, `test`,
`mission`, `run`, `batch`, `campaign`, `analyze`, and `docs`) use an execution
target:

| `--exec` | Behavior outside a container |
|---|---|
| `auto` (default) | Use the named container if it is running, otherwise try a local ROS workspace |
| `container` | Require Docker and the named running container |
| `host` | Require a local ROS installation and workspace |

Inside a SUAVE container, ROS commands run locally regardless of `--exec`.
Each target sources ROS and the workspace overlay when available, then runs
from the workspace root.

Select a nondefault container explicitly, or save it in configuration:

```bash
suave --exec container --container suave_dev test suave_runner
suave config set container_name suave_dev
suave config set exec container
```

Use `--exec container` to prevent fallback to host execution. `--workspace PATH`
selects a **local** workspace, not the container workspace. Configure
`container_workspace` for the latter.

### Run directly on the host

Select host execution explicitly, even if a SUAVE container is running:

```bash
suave --exec host --workspace ~/suave_ws build suave_runner
suave --exec host --workspace ~/suave_ws run --config src/suave/my_runner.yml
```

The CLI sources ROS and the workspace overlay before running the command.
The workspace must contain the SUAVE checkout. For campaigns, the host also
needs the simulation and SITL dependencies. Docker is not required for host
execution.

To make host execution the default:

```bash
suave config set exec host
suave config set host_workspace ~/suave_ws
```

### Paths and workspace selection

The CLI does not translate paths between host and container. Arguments after
`--` also pass through unchanged.

- Relative configuration paths resolve from the execution workspace root.
- Absolute input paths must exist where the wrapped command runs.
- An unquoted `~` expands in the invoking shell, which can produce an invalid
  host-home path for a container command.
- `--latest` searches the target's configured results directory and supported
  timestamped naming patterns. It fails if no matching result exists.

For the default container layout:

```bash
suave run --config src/suave/my_runner.yml
suave campaign resume /home/ubuntu-user/suave/results/2026_09_26_10-15-00 \
  --config src/suave/my_runner.yml
```

Local workspace detection searches above the checkout for a workspace whose
`src/` contains it. For nested Git worktrees, use a container or an explicit
local workspace to avoid colcon discovering duplicate package checkouts.

## Command reference

Run `suave COMMAND --help` or `suave COMMAND SUBCOMMAND --help` for all options.
Global options can appear before or after subcommands, but must precede the
`--` that starts forwarded arguments.

### Docker lifecycle and images

| Command | Purpose |
|---|---|
| `suave docker build [--all] [--tag TAG] [--no-cache]` | Build the headless image. `--all` also builds Kasm and the GUI image |
| `suave docker run [--image IMAGE] [--gpu nvidia\|none]` | Create, start, or reuse the named container |
| `suave docker shell` | Open a ROS-sourced shell in the running container |
| `suave docker status` | Show container state, image, and mounts |
| `suave docker stop [--rm]` | Stop the container and optionally remove it |

Without an explicit image, selection prefers `suave-headless:latest`, then
`suave-headless:dev`, then another local headless tag. If none exists, it offers
to pull `ghcr.io/kas-lab/suave-headless:main`.

`docker run` reuses a running container or restarts a stopped one. It does not
apply new image or mount settings to an existing container. Use `--recreate`
to replace it. Replacement removes the old container, so retain needed files
in bind-mounted directories rather than its writable layer.

Default bind mounts expose the checkout at
`/home/ubuntu-user/suave_ws/src/suave` and host `~/suave/results` at
`/home/ubuntu-user/suave/results`. Source mounts do not replace the image's
existing build and install trees. Rebuild affected packages after changing
C++, interfaces, entry points, or installed resources.

The CLI shares a local X display through a per-container X cookie without
requiring `xhost +`. GUI access requires a usable host display and `xauth`.
Use `--interactive` for a shell container removed on exit, or `--no-mount-src`
to run the code built into the image.

### Additional mounts

Persist mounts for another ROS package or a data directory:

```bash
suave docker mount add ../my_package
suave docker mount add ~/data --to /data
suave docker mount list
suave docker run --recreate
suave build my_package
```

Without `--to`, a mount goes under the container workspace's `src/` directory.
`mount add` requires an existing host path and rejects overlapping destinations.
`mount remove PATH` removes a saved entry by host or container path.

Use `docker run --mount HOST[:CONTAINER]` for a one-time extra mount or
`--no-extra-mounts` to ignore saved entries. Mount changes require recreation
of an existing container.

### Build and test

```bash
suave build suave_runner
suave build -- --cmake-args -DCMAKE_BUILD_TYPE=Release
suave test suave_runner --no-build
suave test suave_monitor --lint
suave test suave_runner -k wilcoxon
```

Without package names, build and test select the built-in SUAVE package set,
not every package in the workspace. Specify external packages explicitly.
`suave_cli` has its own check, `suave self-test`.

`build` invokes `colcon build --symlink-install`. `--clean` first deletes the
selected packages' `build/PKG` and `install/PKG` directories. `test` builds by
default, runs colcon tests, and checks each selected package's test results.
`--no-build` requires existing build artifacts. `--lint` selects linter tests,
and `-k EXPR` filters pytest tests. Arguments after `--` go to colcon.

### Campaigns and batches

| Command | Purpose |
|---|---|
| `suave run [--config FILE]` | Start a campaign using a custom or installed runner configuration |
| `suave run --gui --seed N --run-duration SEC -p KEY:=VALUE` | Override campaign parameters |
| `suave campaign list` | List standalone timestamped campaigns and completed-run counts |
| `suave campaign resume PATH\|--latest --config FILE` | Resume a campaign with its original configuration |
| `suave batch start [--config FILE] [--batch-dir DIR]` | Start a sequence of campaigns |
| `suave batch resume STATE_JSON\|--latest` | Resume unfinished campaigns in a batch |
| `suave batch list` | List timestamped batches and completed-run counts |
| `suave batch diagnose PATH\|--latest [--json]` | Read a batch or campaign's failure report |

`run` without overrides uses the runner launch file. Campaign-specific options
switch to the runner node, retaining the installed configuration unless
`--config` selects another file. `--result-path PATH` sets the parent directory
for a new timestamped campaign. Use `-p gui:=false` to disable GUI explicitly.

Always supply the original configuration when resuming a custom campaign.
Without `--config`, campaign resume warns and uses the installed configuration.
List and `--latest` commands do not discover arbitrary custom directory names.
Use explicit paths for those. See {doc}`run` for batch fail-fast behavior,
readiness checks, recording controls, and recovery safeguards.

### Single mission

```bash
suave mission
suave mission --manager bt --gui
suave mission --manager bt -a bt_executable:=suave_bt_extended -p experiment_logging:=true
```

`mission` replaces the three terminals of the manual run ({doc}`run`). It
starts the runner node with a temporary configuration that holds one
experiment with `num_runs: 1`, launching `suave_bringup mission.launch.py`
with `adaptation_manager:=MANAGER` (default `none`). The runner starts ArduSub
SITL and the simulation, waits for them to be ready, stops all processes when
the mission ends or on Ctrl+C, and writes a normal timestamped campaign folder
under `--result-path` (default `~/suave/results`).

| Option | Purpose |
|---|---|
| `--manager none\|metacontrol\|random\|bt` | Adaptation manager |
| `--gui` | Start Gazebo with its GUI |
| `--result-path PATH` | Parent directory of the campaign folder |
| `--run-duration SEC` | Maximum seconds to wait for the mission to finish |
| `-a`, `--launch-arg KEY:=VALUE` | Extra `mission.launch.py` argument, repeatable |
| `-p`, `--param KEY:=VALUE` | Runner parameter override, repeatable |

The other runner parameters keep their node defaults, which differ from the
installed `runner_config.yml`: the vehicle starts at `x=-17.0`, `y=2.0`, the
mission uses `suave_missions/config/mission_config.yaml`, and no thruster or
water-visibility perturbations are injected. Node output is silent unless
`-p experiment_logging:=true`. The SITL console is not opened; its output goes
to the run's `ardupilot.log`. Run the three commands by hand when you need the
interactive console.

### Analysis

```bash
suave analyze summarize -- /home/ubuntu-user/suave/results/CAMPAIGN --latex
suave analyze wilcoxon -- --ros-args --params-file src/suave/my_analysis.yml
suave analyze wilcoxon-exp exp1 /home/ubuntu-user/suave/results/CAMPAIGN --correction holm
suave analyze wilcoxon-batch -- /home/ubuntu-user/suave/results/batches/BATCH --correction holm
suave analyze qq-batch -- /home/ubuntu-user/suave/results/batches/BATCH
```

| Command | Runs |
|---|---|
| `summarize` | `ros2 run suave_runner summarize_results` |
| `wilcoxon` | `ros2 run suave_runner wilcoxon_analysis` |
| `mann-whitney` | `ros2 run suave_runner mann_whitney_analysis` |
| `wilcoxon-exp EXPERIMENT RESULTS_ROOT` | `ros2 launch suave_runner EXPERIMENT_analysis_launch.py` |
| `wilcoxon-batch` | `suave_runner/suave_runner/analysis/wilcoxon_analysis_batch.py` |
| `qq` | `suave_runner/suave_runner/analysis/qq_plot.py` |
| `qq-batch` | `suave_runner/suave_runner/analysis/qq_plot_batch.py` |
| `latex-tables` | `suave_runner/suave_runner/latex/wilcoxon_latex_tables_batch.py` |
| `requirements` | `suave_requirements/scripts/plot_requirement_fulfillment.py` |
| `representative-run` | `suave_requirements/scripts/export_representative_run_tikz.py` |
| `requirements-tikz` | `suave_requirements/scripts/export_requirement_fulfillment_tikz.py` |
| `footprint-alternatives` | `suave_requirements/scripts/export_search_footprint_alternatives.py` |

Arguments after `--` go to the executable or script unchanged. The scripts run
with `python3` from the SUAVE checkout (`container_src_dir` in a container, the
CLI's checkout otherwise), so relative paths are relative to the checkout. The
`suave_requirements` scripts are not installed and exist only in the checkout.
`wilcoxon-exp` takes one of the bundled experiments (`exp1` to `exp3`,
`extended_exp1` to `extended_exp3`) and the campaign folder, plus optional
`--correction holm|none` and `--output-root DIR`; extra launch arguments go
after `--`. See {doc}`results` for the analysis assumptions and input
requirements.

### Documentation

```bash
suave docs build
suave docs build --output /tmp/suave-docs -- -j auto
```

`docs build` builds the HTML documentation the way CI does: from the SUAVE
checkout, with ROS and the workspace sourced (the API pages import the
packages), with `sphinx-build -W --keep-going -E -a docs/source docs/build`.
It first removes the generated `docs/source/_autosummary` pages, which can hold
stale pages of removed functions. On first use, it installs the pinned
`docs/requirements.txt` into `~/.cache/suave/docs-python` where the command
runs, and reinstalls them when that file changes; this needs network access
and `pip`. `--output DIR` is relative to the checkout. Arguments after `--` go
to `sphinx-build`.

## Configuration

```bash
suave config show
suave config set container_name suave_dev
suave config unset container_name
suave config path
suave config init
```

`show` reports effective values and their sources. `path` identifies the active
file. `init` runs interactive setup. Settings resolve in this order:

```text
command-line flag > environment variable > configuration file > default
```

Each checkout stores host settings in `.config/host.ini` and container settings
in `.config/container.ini`. Both files are gitignored. Changing host settings
does not change the configuration used by a CLI invoked inside the container.

| Setting | Default | Flag or environment override |
|---|---|---|
| `exec` | `auto` | `--exec`, `SUAVE_EXEC` |
| `container_name` | `suave` | `--container`, `SUAVE_CONTAINER_NAME` |
| `image` | Auto-select | `docker run --image`, `SUAVE_IMAGE` |
| `gpu` | `nvidia` | `docker run --gpu`, `SUAVE_GPU` |
| `run_mode` | `detached` | `docker run --detach` or `--interactive` |
| `mount_src` | `true` | `docker run --[no-]mount-src` |
| `mount_results` | `true` | `docker run --[no-]mount-results` |
| `host_results_dir` | `~/suave/results` | `docker run --results-dir` |
| `extra_mounts` | Empty | `docker mount add/remove/list` |
| `host_workspace` | Auto-detect | `--workspace`, `SUAVE_WORKSPACE` |
| `ros_setup` | `/opt/ros/humble/setup.bash` | `config set` |
| `container_workspace` | `/home/ubuntu-user/suave_ws` | `config set` |
| `container_src_dir` | `/home/ubuntu-user/suave_ws/src/suave` | `config set` |
| `container_results_dir` | `/home/ubuntu-user/suave/results` | `config set` |

Boolean settings accept `true/false`, `yes/no`, `on/off`, or `1/0`.
`host_results_dir` and `container_results_dir` control result mounts and lookup.
They do not override a runner's `result_path`. Set that campaign parameter
separately when changing the output destination.

## Command previews and automation

```bash
suave --exec container --container suave_dev --dry-run build suave_runner
suave batch start --config src/suave/my_batch.yml --dry-run-batch
```

Global `--dry-run` previews wrapped commands but can still inspect Docker state.
It requires a resolvable execution target for ROS commands. `--dry-run-batch`
instead runs the batch node in its dry-run mode, which prints campaign commands
without executing them or checking installed campaign configuration files.

Use `--verbose` to print commands during execution. `--yes` approves image pulls
without a prompt. Noninteractive invocations do not offer setup or write an
initial configuration automatically. Explicit `config set` commands still write
configuration.

The CLI normally returns the wrapped command's exit code. `suave test` also
checks recorded test results and returns nonzero if a selected package failed.
Explicit path guards, such as those for custom runner configurations, return
2 when the path is missing. Forwarded arguments retain the wrapped tool's
validation and exit behavior.

## Troubleshooting

| Symptom | Action |
|---|---|
| No execution target or named container is stopped | Run `suave docker status`, start the container, or select its actual name with `--container` |
| Configuration or result path not found | Check the path inside the selected target and its workspace root |
| NVIDIA runtime unavailable | Install the NVIDIA Container Toolkit or use `docker run --gpu none` |
| Changed image or mounts have no effect | Preserve needed container-local data, then use `docker run --recreate` |
| GUI cannot open the display | Check host `DISPLAY`, `xauth`, and user permissions. Recreate after changing displays |
| No local workspace found | Set `--workspace` to a valid colcon workspace or select container execution |
| Mount destination overlaps another mount | Select a distinct destination with `--to` |
| `--latest` finds no results | Check configured result directories and use an explicit path for custom names |

Use `suave config show` to inspect settings before changing them. For CLI
regressions, run `suave self-test`. It runs the CLI's own tests and linters
without ROS or colcon.
