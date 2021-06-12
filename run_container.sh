#!/bin/sh

COMMAND=/bin/bash

xhost + # allow connections to X server

docker run -it \
    --name="drone_simulator"\
    --gpus all \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --volume="$PWD:/root/share"\
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --privileged \
    --network=host \
    -p 2222:2222 \
    kroszyk/tum-simulator-gpu-support \
    $COMMAND\
    
    --device=/dev/input/js0:/dev/input/js0 \
    --device=/dev/input/js1:/dev/input/js1 \
    --device=/dev/input/js2:/dev/input/js2 \
    --device=/dev/input/js3:/dev/input/js3 \
    

