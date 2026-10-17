# suave CLI

One command for everyday SUAVE work: Docker images and the container, colcon build/test,
experiment campaigns and batches, and the analysis scripts.

The CLI wraps existing tools and does not replace them. Each subcommand builds a `docker`,
`colcon`, `ros2 launch` or `ros2 run` command line and runs it. `build_docker_images.sh`, the
launch files and the runner configs are still the source of truth. Add `--dry-run` to any
command to see what it would run.

- Plain Python (>= 3.10, stdlib only). It runs from the checkout and needs no build or `pip`.
- It never imports ROS, so it also works on a host that has only Docker.
- `suave --help` and `suave COMMAND --help` are the full option reference. This file covers
  the concepts, workflows and pitfalls.

## Contents

- [Put it on PATH](#put-it-on-path)
- [Quick start](#quick-start)
- [Commands](#commands)
- [Where ROS commands run](#where-ros-commands-run)
- [Paths and the container](#paths-and-the-container)
- [Configuration](#configuration)
- [Extra mounts](#extra-mounts)
- [Scripting and automation](#scripting-and-automation)
- [Troubleshooting](#troubleshooting)
- [Development](#development)

## Put it on PATH

Pick one:

| Method | Notes |
|---|---|
| `source <repo>/env.sh` | bash only; add it to `~/.bashrc` to keep it |
| build the workspace, then `source install/setup.bash` | the `suave_cli` colcon package only installs a hook that sets `SUAVE_ROOT` and `PATH` |
| `ln -s <repo>/suave_cli/bin/suave ~/.local/bin/suave` | works in any shell |

Without any of these, call `<repo>/suave_cli/bin/suave` directly. Inside the SUAVE images
`suave` is already on `PATH`, and `SUAVE_CLI_CONTEXT=container` tells it that it runs in a
container.

**Which checkout is used:** `--suave-root PATH`, else `$SUAVE_ROOT`, else the checkout that
contains the `suave` launcher that was run. That checkout's `.config/` holds the settings,
and it is the tree `suave docker build` builds from (it must contain
`build_docker_images.sh` and `docker/versions.env`).

## Quick start

A typical first session on a host with Docker:

```bash
suave docker build              # build suave-headless:latest from this checkout
suave docker run                # start container 'suave' in the background
suave test suave_runner         # colcon build + test inside the container
suave run                       # one campaign with the installed runner_config.yml
suave campaign list             # result folders and how many runs finished
suave docker shell              # interactive shell in the container, ROS sourced
suave docker stop               # stop it (add --rm to remove it)
```

On the first interactive run the CLI offers a short setup. You can skip it: the defaults
work, and `suave config init` asks the questions again later.

## Commands

### `suave docker` (host only)

| Command | What it does |
|---|---|
| `docker build [--all] [--tag T] [--no-cache]` | Builds `suave-headless:latest` with the SHAs pinned in `docker/versions.env`. `--all` also builds the Kasm GUI images (`kasm-jammy:dev`, `suave:dev`). `--tag` sets the tag for all of them. |
| `docker run` | Starts the container, or reuses it (see below). Arguments after `--` go to `docker run`, for example `-- --network host`. |
| `docker shell` | Opens a login shell in the running container with ROS and the workspace sourced. |
| `docker stop [--rm]` | Stops the container, and removes it with `--rm`. |
| `docker status` | Shows whether the container exists and is running, and what it mounts. |
| `docker mount add\|remove\|list` | Manages saved extra bind mounts. See [Extra mounts](#extra-mounts). |

What `suave docker run` does depends on the container's current state:

| Container state | Result |
|---|---|
| running | reused as is |
| exists, stopped | restarted with `docker start`; image and mount changes are **not** applied (it warns if they differ) |
| missing, or `--recreate` given | created with the selected image and mounts |

When no image is given (`--image` or the `image` setting), the CLI uses the first match:
1. `suave-headless:latest`
2. `suave-headless:dev`
3. any other local `suave-headless` tag (with a warning)
4. `ghcr.io/kas-lab/suave-headless:main`, pulled after you confirm, or with `--yes`

By default the container gets these mounts:
- this checkout at `/home/ubuntu-user/suave_ws/src/suave`
- `~/suave/results` at `/home/ubuntu-user/suave/results`, created if missing
- any saved extra mounts

The container also gets access to the host display and GPU:

| Setup | docker run options |
|---|---|
| always | `-v /etc/localtime:/etc/localtime:ro` |
| `gpu=nvidia` (default) | `--gpus all --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -v /dev/dri:/dev/dri` |
| `DISPLAY` is set | `-e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix`, plus the X cookie below |

Use `--gpu none` (or `suave config set gpu none`) on machines without an NVIDIA GPU or
without the NVIDIA Container Toolkit. If `gpu=nvidia` but Docker has no `nvidia` runtime,
`docker run` stops with an error before creating anything.

**Display access without `xhost +`:** the CLI does not change the X server's access list.
Instead, it gives the container its own copy of the display's X cookie:
- `xauth nlist $DISPLAY` reads the cookie. The CLI rewrites it so that it matches any
  hostname, because the container has its own hostname.
- It stores the result in `~/.cache/suave/xauth/<container>/Xauthority` (file 0600, folder
  0700), and mounts only that container's folder, read-only, at `/tmp/.suave-xauth`.
- `XAUTHORITY` points at the file inside the container.
- Every `suave docker run` refreshes the cookie, including when it reuses a container, so a
  new login session works without `--recreate`.

A container can therefore only use the display whose cookie it was given. This only
restricts anything while the X server's access control is on: if `xhost` prints
`access control disabled`, an earlier `xhost +` is still active, and `xhost -` turns it
back on. The container
user has uid 1000, so the cookie can only be read when your host uid is also 1000; the CLI
warns otherwise. `DISPLAY` is fixed when the container is created: after switching
displays, run `suave docker run --recreate` (the CLI warns about the mismatch).

The image's `build/` and `install/` come from the copy built into it. After changing C++,
messages or entry points in the mounted checkout, run `suave build`.

Other run modes:
- `--interactive` opens a shell and removes the container when you exit.
- `--no-mount-src` uses the code built into the image, for example to test a released
  image.

### `suave build` and `suave test`

Without package names both commands cover every SUAVE package: `suave`, `suave_base`,
`suave_bringup`, `suave_bt`, `suave_metacontrol`, `suave_metrics`, `suave_missions`,
`suave_monitor`, `suave_msgs`, `suave_none`, `suave_random`, `suave_requirements`,
`suave_runner`, `suave_tools`.
`suave_cli` is not in the list; use `suave self-test` for it.

```bash
suave build suave_runner --clean        # rm -rf build/ and install/ for it, then build
suave build -- --cmake-args -DCMAKE_BUILD_TYPE=Release
suave test suave_monitor --lint         # linters only (flake8, pep257, copyright, ...)
suave test suave_runner -k wilcoxon     # pytest -k filter
suave test suave_runner --no-build      # skip the build step
```

`build` runs `colcon build --symlink-install --packages-select ...`. `test` builds, then runs
`colcon test`, then runs `colcon test-result` for each package. It exits non-zero if any
selected package failed, or if a package has no `build/` folder (which can happen with
`--no-build`).

### `suave run`, `suave batch`, `suave campaign`

These wrap the `suave_runner` and `run_batch` nodes. Their parameters are documented in
`suave_runner/README.md`.

| Command | Runs |
|---|---|
| `run` | `ros2 launch suave_runner suave_runner_launch.py` (installed `runner_config.yml`) |
| `run --config F --seed N --run-duration S --result-path P --gui -p K:=V` | `ros2 run suave_runner suave_runner --ros-args --params-file F -p ...` |
| `batch start [--config F] [--batch-dir D] [--fail-fast] [--dry-run-batch]` | `run_batch_launch.py`, or `ros2 run ... run_batch` when an option is given |
| `batch resume STATE_JSON \| --latest` | `run_batch` with `resume_state_file` |
| `batch list` | every `<results>/batches/batch_*/state.json` with its progress |
| `campaign resume RESULT_PATH \| --latest [--config F]` | `suave_runner` with `resume_result_path` |
| `campaign list` | campaign folders under the results folder with their finished runs |

Things to keep in mind:
- Any `run` option switches from `ros2 launch` to `ros2 run`. The config file is still the
  installed `runner_config.yml` unless `--config` is given, so both forms read the same
  config.
- Pass `campaign resume` the **same** `--config` the campaign started with. Without it the
  CLI warns and falls back to the installed `runner_config.yml`.
- `--dry-run-batch` sets the `run_batch` node's `dry_run` parameter, which checks the
  campaigns without running them. It is not the global `--dry-run`, which only prints
  commands.
- `--latest` picks the newest batch or campaign in the results folder, looking inside
  wherever the command runs.

### `suave analyze`

`suave analyze wilcoxon|mann-whitney|summarize` runs `ros2 run suave_runner
wilcoxon_analysis|mann_whitney_analysis|summarize_results`. Everything after `--` goes to
the script unchanged:

```bash
suave analyze summarize -- --help
suave analyze wilcoxon -- --ros-args --params-file my_analysis.yml
```

### `suave config` and `suave self-test`

| Command | What it does |
|---|---|
| `config show` | every setting, its value and where the value came from |
| `config set KEY VALUE` / `config unset KEY` | store or remove one setting in the active config file |
| `config init` | run the setup questions again (needs a terminal) |
| `config path` | print the active config file |
| `self-test` | the CLI's unit tests and linters; no ROS or colcon needed |

### Global options

These go before or after the command: `--exec`, `--container NAME`, `--workspace PATH`,
`--suave-root PATH`, `--dry-run`, `-y/--yes` (answer yes to prompts such as image pulls),
and `-v/--verbose` (print each command before running it).

## Where ROS commands run

`build`, `test`, `run`, `batch`, `campaign` and `analyze` need ROS. The `exec` setting
(`--exec auto|container|host`, default `auto`) chooses where they run:

| Mode | Runs in | Fails when |
|---|---|---|
| `auto` | the container named by `container_name` if it is running, otherwise the host workspace | neither is available |
| `container` | `docker exec <container> bash -lc ...` in `container_workspace` | Docker is missing or the container is not running |
| `host` | `bash -lc ...` in the host workspace | `ros_setup` or the workspace is missing |

The host workspace is `host_workspace` if it is set. Otherwise it is the nearest parent
folder whose `src/` contains the checkout. Every command sources `ros_setup` and, if it
exists, `install/setup.bash`, then runs from the workspace root.

Inside a SUAVE container, commands always run directly, whatever `exec` says.

**Git worktrees:** a worktree nested under a workspace's `src/` (for example under
`.claude/worktrees/`) also sits inside the enclosing workspace. Host mode then finds that
workspace, and colcon sees the SUAVE packages twice. In that case pass an explicit
`--workspace`, or use the container.

## Paths and the container

The CLI does **not** translate paths. The values of `--config`, `--result-path`,
`--batch-dir`, `STATE_JSON` and `RESULT_PATH` are passed as they are and read wherever the
command runs, from the workspace root:

- In container mode, a host path does not exist. Your shell expands `~/suave/results/x` to
  your host home, but in the container the results live at `/home/ubuntu-user/suave/results`.
- A relative path such as `--config my_runner.yml` is resolved against the workspace root,
  not your current directory.
- A config file must be in a folder the container can see: the mounted checkout, the
  results folder, or an [extra mount](#extra-mounts).
- `--latest` always works, because the lookup runs where the command runs.

If a path does not exist, the command prints `not found: <path>` and exits with code 2.

```bash
# container mode: use container paths, or --latest
suave campaign resume /home/ubuntu-user/suave/results/2026_09_26_10-15-00 \
    --config src/suave/my_runner.yml
suave batch resume --latest
```

## Configuration

Settings are stored for each checkout in `.config/host.ini` on the host and
`.config/container.ini` in the container. Both files are gitignored. A value is taken from
the first of these that sets it:

    command-line flag  >  environment variable  >  config file  >  built-in default

`suave config show` prints each value and its source.

| Key | Default | Flag / env |
|---|---|---|
| `exec` | `auto` | `--exec` / `SUAVE_EXEC` |
| `container_name` | `suave` | `--container` / `SUAVE_CONTAINER_NAME` |
| `image` | empty (auto-select) | `--image` / `SUAVE_IMAGE` |
| `run_mode` | `detached` | `--detach`, `--interactive` |
| `gpu` | `nvidia` | `--gpu nvidia\|none` / `SUAVE_GPU` |
| `mount_src` | `true` | `--[no-]mount-src` |
| `mount_results` | `true` | `--[no-]mount-results` |
| `host_results_dir` | `~/suave/results` | `--results-dir` |
| `extra_mounts` | empty | `suave docker mount ...`; `--mount`, `--no-extra-mounts` on `docker run` |
| `host_workspace` | empty (auto-detect) | `--workspace` / `SUAVE_WORKSPACE` |
| `ros_setup` | `/opt/ros/humble/setup.bash` | |
| `container_workspace` | `/home/ubuntu-user/suave_ws` | |
| `container_src_dir` | `/home/ubuntu-user/suave_ws/src/suave` | |
| `container_results_dir` | `/home/ubuntu-user/suave/results` | |

Value formats:
- Booleans accept `true/yes/on/1` and `false/no/off/0`.
- `extra_mounts` is a comma- or newline-separated list of `HOST:CONTAINER` entries.
- The `container_*` defaults match the headless image. Change them only for a custom image.

In the host config, setup asks about the container, the image, the mounts and the
workspace. In the container config it asks only about `exec` and `host_workspace`.

Example `.config/host.ini`:

```ini
[suave]
exec = container
container_name = suave_dev
host_results_dir = ~/experiments/results
```

## Extra mounts

Extra mounts add more host folders to the container, such as another ROS package to build
next to SUAVE, or a data folder:

```bash
suave docker mount add ../my_package          # -> <container_workspace>/src/my_package
suave docker mount add ~/data --to /data      # any absolute container path
suave docker mount list                       # default and saved mounts
suave docker mount remove ../my_package       # by host path or container path
suave docker run --recreate                   # mounts only apply when the container is created
suave build my_package
```

- `mount add` stores the host path as an absolute path. It rejects a host path that does
  not exist, and a destination that overlaps another mount.
- `suave docker run --mount HOST[:CONTAINER]` adds a mount for that run only.
  `--no-extra-mounts` skips the saved mounts.
- Extra mounts come after the checkout and results mounts.

## Scripting and automation

- **`--dry-run`** prints the commands instead of running them, but still asks Docker which
  containers and images exist. `--exec container --dry-run` therefore still fails when the
  container is not running.
- **Without a terminal** (CI, scripts, agents) the CLI never prompts and never writes a
  config. It prints a one-line tip if no config exists.
- **`--yes`** approves an image pull. Without it, a run with no terminal that needs a pull
  fails with a message.
- **Exit codes:**
  - The wrapped command's exit code is passed through.
  - `suave test` fails if any selected package failed.
  - A missing path gives exit code 2.
  - CLI errors print `error: ...` and exit non-zero.
- Set `SUAVE_CONTAINER_NAME` or pass `--container` when the container is not called `suave`.

## Troubleshooting

| Message | Fix |
|---|---|
| `nowhere to run ROS commands: container '...' is not running and ...` | `suave docker run`, or set up host runs with `suave config init` / `--workspace` |
| `container '...' is not running; start it with: suave docker run` | start it, or pass `--container` if it has another name |
| `image ... is not available locally; pass --yes to pull it` | `suave docker build`, or add `--yes` |
| `container '...' uses image ... with mounts ...; ... Use --recreate` | the existing container was created with other settings: `suave docker run --recreate` |
| `mount destination ... overlaps ...` | choose another destination with `--to`; `/tmp/.X11-unix`, `/tmp/.suave-xauth`, `/dev/dri` and `/etc/localtime` are taken too |
| `Docker has no nvidia runtime` | install the NVIDIA Container Toolkit, or use `--gpu none` / `suave config set gpu none` |
| GUI apps fail with `cannot open display` or `Authorization required` | check that `DISPLAY` was set when the container was created (`suave docker status`, or `--recreate`), that `xauth` is installed on the host, and that your uid is 1000 |
| `DISPLAY=... is a network display` | SSH X forwarding (`localhost:10.0`) cannot be reached through the socket mount; use a local display |
| `not found: <path>` | the path does not exist where the command runs (see [Paths and the container](#paths-and-the-container)) |
| `ROS setup file ... was not found` | install ROS on the host, set `ros_setup`, or use the container |
| `no colcon workspace contains ... in its src/ folder` | set `host_workspace` or pass `--workspace` |
| `... is not a SUAVE checkout` | `--suave-root` or `SUAVE_ROOT` points at the wrong folder |
| `suave config init needs an interactive terminal` | use `suave config set KEY VALUE` instead |

## Development

```bash
suave self-test                                   # unit tests + linters, no colcon
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q suave_cli/test   # from the repo root
colcon test --packages-select suave_cli           # same tests + ament linters
```

The code is in `suave_cli/suave_cli/`. Each command group has its own module (`container.py`,
`ros_build.py`, `runner.py`, `analyze.py`, `config.py`). `targets.py` chooses where ROS
commands run.
