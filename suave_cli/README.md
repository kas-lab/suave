# suave CLI

One command for everyday SUAVE work: Docker images and container, colcon build/test,
experiment campaigns and batches. It is plain Python (stdlib only) and needs no build.

## Put it on PATH (pick one)

- `source <repo>/env.sh` (bash; e.g. add it to `~/.bashrc`)
- build the workspace and `source install/setup.bash` (the `suave_cli` package only
  installs a hook that sets `SUAVE_ROOT` and `PATH`)
- `ln -s <repo>/suave_cli/bin/suave ~/.local/bin/suave`

Inside the SUAVE images `suave` is already on `PATH`.

## Quick start

    suave docker build          # suave-headless:latest
    suave docker run            # background container 'suave', checkout + ~/suave/results mounted
    suave test suave_runner     # build + test in the container
    suave run                   # experiment runner with the installed runner_config.yml
    suave batch resume --latest

`suave --help` and `suave COMMAND --help` list every option with examples.
`--dry-run` prints the commands instead of running them.

## Where ROS commands run

`--exec auto|container|host` (setting `exec`, default `auto`): `auto` uses the running
container, otherwise a local colcon workspace (the one whose `src/` contains this checkout,
or `host_workspace`), otherwise explains what to start. Inside the container commands
always run directly.

## Configuration

Stored per checkout in `.config/host.ini` (host) and `.config/container.ini` (container),
both gitignored. The first interactive run offers a short setup; `suave config init`
repeats it. Precedence: flag > environment variable > config file > default.

| Key | Default | Flag / env |
|---|---|---|
| exec | auto | `--exec` / `SUAVE_EXEC` |
| container_name | suave | `--container` / `SUAVE_CONTAINER_NAME` |
| image | auto-select | `--image` / `SUAVE_IMAGE` |
| run_mode | detached | `--detach`, `--interactive` |
| mount_src, mount_results | true | `--[no-]mount-src`, `--[no-]mount-results` |
| host_results_dir | ~/suave/results | `--results-dir` |
| host_workspace | auto-detect | `--workspace` / `SUAVE_WORKSPACE` |
| ros_setup | /opt/ros/humble/setup.bash | |
| container_results_dir, container_src_dir, container_workspace | headless image paths | |

`suave config show` prints every value and where it came from.

## Development

    suave self-test                                   # unit tests + linters, no colcon
    colcon test --packages-select suave_cli           # same tests + ament linters
