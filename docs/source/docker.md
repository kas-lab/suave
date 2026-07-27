# Use SUAVE with Docker

You can pull and run the exemplar as a Docker container using the following command. Keep in mind you need to have [Docker](https://docs.docker.com/get-docker/) installed on your computer and running.

In a terminal on your computer run:
```Bash
docker run -it --shm-size=512m -p 6901:6901 -e VNC_PW=password --security-opt seccomp=unconfined ghcr.io/kas-lab/suave:main
```

Optionally you can add the parameter `-v <absolute_path_host_compute>:/home/kasm-user/suave/results` to save the results into your computer, replace `<absolute_path_host_compute>` with the absolute path of where you want the data to be saved in your computer, e.g:

```Bash
docker run -it --shm-size=512m -v $HOME/suave_results:/home/kasm-user/suave/results -p 6901:6901 -e VNC_PW=password --security-opt seccomp=unconfined ghcr.io/kas-lab/suave:main
```

**SEAMS2023:** To use the docker image used in the SEAMS2023 paper, replace `ghcr.io/kas-lab/suave:main` with `ghcr.io/kas-lab/suave:seams2023`.

Once the container is up and running, you can interface with it through your web browser. The container will be hosted locally at the port specified, in this case 6901. So in your browser, go to
`http://localhost:6901`.

A dialog will request a username and password, these are shown below, with the password being specifiable in the run command.

 - **User** : `kasm_user`
 - **Password**: `password`

<!-- Now you can proceed to [run the exemplar](#run-suave). -->

## Headless docker image

We also provide a docker image without a VNC/Web interface, so you can run experiments directly from your terminal without logging in to the web interface.

Pull image:
```Bash
docker pull ghcr.io/kas-lab/suave-headless:main
```

Build the headless image locally from your current checkout:
```Bash
docker build -t suave-headless:dev -f docker/dockerfile-suave-headless .
```

### Non-interactive (pass command directly)

Run an experiment campaign and save results to your host machine:

```Bash
docker run -it --shm-size=512m \
  -v $HOME/suave_results:/home/ubuntu-user/suave/results \
  ghcr.io/kas-lab/suave-headless:main \
  ./src/suave/runner/headless_runner.sh false metacontrol time 2
```

The runner arguments are: `[true|false] [metacontrol|random|none|bt] [time|distance] <runs>`.

### Interactive (open a shell, then run inside)

Run the local image without GPU:
```Bash
docker run -it --rm --name suave_runner -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro suave-headless:dev
```

Run the local image with NVIDIA GPU support. Install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) first.
```Bash
docker run -it --rm --gpus all --runtime=nvidia --name suave_runner -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro suave-headless:dev
```

If you want the image to have access to the host GUI, run the following command before starting the container:
```Bash
xhost +
```

Once inside the container, start an experiment with the ROS 2 runner:

```Bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p gui:=False \
  -p experiments:='[
    "{\"experiment_launch\": \"ros2 launch suave_bringup mission.launch.py adaptation_manager:=bt\", \
      \"num_runs\": 2, \
      \"adaptation_manager\": \"bt\", \
      \"mission_name\": \"suave\"}"
  ]'
```

## Build Docker images locally
To build the docker images locally, run:

```Bash
./build_docker_images.sh
```

This builds `kasm-jammy:dev`, `suave:dev`, and `suave-headless:dev` from the repository root.
