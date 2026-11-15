# Use SUAVE with Docker

SUAVE provides two images:

| Image | Use |
|---|---|
| `ghcr.io/kas-lab/suave-headless:main` | Terminal-based development and experiments. The SUAVE CLI uses this image |
| `ghcr.io/kas-lab/suave:main` | Browser-based desktop (Kasm VNC), for trying SUAVE without a local display setup |

You need [Docker](https://docs.docker.com/get-docker/) installed and running.

## Headless image with the SUAVE CLI

The SUAVE CLI is the recommended way to start and use the container. From the
root of a SUAVE checkout:

```bash
source env.sh
suave docker run
suave run
```

`suave docker run` starts a container named `suave`. If no SUAVE image exists
locally, it offers to pull `ghcr.io/kas-lab/suave-headless:main`. The
container mounts the checkout at `/home/ubuntu-user/suave_ws/src/suave` and
the host's `~/suave/results` at `/home/ubuntu-user/suave/results`, so results
stay on the host. `suave run` then starts a campaign inside the container. See
{doc}`run` for campaign configuration.

Other common commands:

```bash
suave docker run --gpu none   # without NVIDIA GPU access
suave docker shell            # open a shell with ROS and the workspace sourced
suave docker status
suave docker stop --rm
```

GPU access is requested by default and requires the
[NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).
The CLI shares the host display with a per-container X cookie, so Gazebo's GUI
works without `xhost +`. See {doc}`cli` for all options, extra mounts, and
configuration.

### Equivalent Docker commands

For reference, without GPU or display access, `suave docker run` and
`suave docker shell` correspond to:

```bash
docker run -d --name suave \
  -v "$PWD:/home/ubuntu-user/suave_ws/src/suave" \
  -v "$HOME/suave/results:/home/ubuntu-user/suave/results" \
  ghcr.io/kas-lab/suave-headless:main sleep infinity
docker exec -it suave bash
```

Run these from the checkout root. To run a campaign without opening a shell:

```bash
docker exec suave suave run
```

GPU access adds `--gpus all --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=all
-e NVIDIA_DRIVER_CAPABILITIES=all -v /dev/dri:/dev/dri`. Display access adds
`-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix` plus X authentication,
which the CLI configures for you. Avoid `xhost +`: it disables access control
for the whole X server.

The source mount replaces the image's copy of the checkout, but not its
`build/` and `install/` folders. Run `suave build` after changing C++,
messages, entry points, or installed files.

## Browser-based image

The Kasm image runs a desktop that you open in a web browser:

```bash
docker run -it --name suave --shm-size=512m -p 6901:6901 -e VNC_PW=password \
  --security-opt seccomp=unconfined \
  -v "$HOME/suave/results:/home/kasm-user/suave/results" \
  ghcr.io/kas-lab/suave:main
```

Then open `http://localhost:6901` and log in with user `kasm_user` and the
password set by `VNC_PW` (`password` above). Results are saved to
`~/suave/results` on the host. Remove the `-v` option to keep them only inside
the container. For the image used in the SEAMS 2023 paper, see
[Reproduce the SEAMS 2023 artifact](installation.md#reproduce-the-seams-2023-artifact).

## Build images locally

```bash
suave docker build          # suave-headless:latest
suave docker build --all    # also the Kasm base and GUI images
```

Without the CLI, `./build_docker_images.sh` builds `kasm-jammy:dev`,
`suave:dev`, and `suave-headless:dev` from the repository root, and
`docker build -t suave-headless:dev -f docker/dockerfile-suave-headless .`
builds only the headless image. Select a local image with
`suave docker run --image suave-headless:dev`.
