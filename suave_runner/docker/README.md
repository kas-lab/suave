## Run docker image


### With GUI and nvidia

```
xhost +local:root  # Allow Docker to access the X server
```

```
docker run -it --rm \
  --gpus all \
  --runtime=nvidia \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  suave_runner
```

