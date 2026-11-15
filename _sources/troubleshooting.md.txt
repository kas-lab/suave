# Troubleshooting

## Common problems

| Symptom | Cause and fix |
|---|---|
| A run fails with `readiness_timeout` or `readiness_missing_service` | SITL, the simulation, or the experiment did not start in time. Run `suave batch diagnose --latest` and read the reported attempt logs, starting with `ardupilot.log`. Check that `required_services` lists services the manager actually provides. See [Monitor and diagnose](run.md#monitor-and-diagnose) |
| The runner refuses to start a run because of a live `gz sim` process or a bound UDP port 9002 | A simulation or SITL process from an earlier run is still alive. Stop it, or restart the container, then resume. See [Recovery safeguards](run.md#recovery-safeguards) |
| MAVROS reports a GeographicLib exception at startup (local installation) | The geoid datasets are missing. Run `install_geographiclib_datasets.sh` as described in [Install SUAVE workspace](installation.md#install-suave-workspace) |
| The Gazebo window does not open from the container | Start the container with `suave docker run`, which shares the display with an X cookie, and recreate it after changing displays. See the [CLI troubleshooting](cli.md#troubleshooting) |
| The browser desktop of the Kasm image is blank or crashes | Start the container with `--shm-size=512m`, as in [Browser-based image](docker.md#browser-based-image) |
| A change to C++, messages, entry points, or configuration files has no effect | The installed copy is stale. Rebuild the package, for example `suave build suave_runner` |

## MAVROS and ArduSub integration

Inspect SITL and MAVROS logs before changing dependencies. Use the tested
ArduSub commit `571e8c7bd3793fce1bc5184a2f6586feb8a616e5` and MAVROS `2.14.0`
as the starting point. ArduSub, MAVROS, and the Gazebo plugin must remain
compatible. Do not delete an existing checkout to change versions.

### ArduSub

Inspect the checkout and its submodules first:

```bash
cd ~/ardupilot
git status --short
git submodule foreach --recursive git status --short
```

Save local changes before switching versions. To restore the tested revision:

```bash
git fetch origin
git checkout 571e8c7bd3793fce1bc5184a2f6586feb8a616e5
git submodule update --init --recursive
```

Follow the [ArduSub installation instructions](installation.md#install-ardusub)
for prerequisites, then rebuild SITL when starting it. Do not use `-N` for the
first start after changing the source revision.

### MAVROS

Inspect and preserve local changes, then check out the tested version:

```bash
cd ~/suave_ws/src/mavros
git status --short
```

After saving local changes:

```bash
git fetch origin --tags
git checkout 2.14.0
```

For another version, update the MAVROS entry in `suave.repos` and explicitly
check out that same tag, commit, or branch in the existing repository.
Editing the manifest does not change an existing checkout. `vcs pull src`
pulls the currently checked-out branches and does not apply manifest versions.

Rebuild the workspace after changing MAVROS:

```bash
cd ~/suave_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

For campaign failures, use the [runner diagnostics](run.md#monitor-and-diagnose)
and inspect the reported attempt logs before retrying.
