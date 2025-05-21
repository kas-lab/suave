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

We also provide a docker image without VCN/Web interface, so you can just run the experiments directly from your terminal without needing to log in into the web interface.

Pull image:
```Bash
docker pull ghcr.io/kas-lab/suave-headless:main
```

Run image (without gpu):
```Bash
docker run -it --rm --name suave_runner -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro -v $HOME/suave_ws/src/suave:/home/ubuntu-user/suave_ws/src/suave suave-headless
```

Run image (with nvidia)(don't forget to install nvidia docker toolkit)[https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html]:
```Bash
docker run -it --rm --gpus all --runtime=nvidia --name suave_runner -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro -v $HOME/suave_ws/src/suave:/home/ubuntu-user/suave_ws/src/suave suave-headless
```

You can try by running:

```Bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p gui:=False \
  -p experiments:='[
    "{\"experiment_launch\": \"ros2 launch suave_bt suave_bt.launch.py\", \
      \"num_runs\": 2, \
      \"adaptation_manager\": \"bt\", \
      \"mission_name\": \"suave\"}"
  ]'
```

If you want the image to have acess to the host GUI run the following command before running the image:
```Bash
xhost +
```

## Build Docker images locally
To build the docker images locally, run:

```Bash
./build_docker_images.sh
```
