#!/usr/bin/bash
xhost +
sudo docker run -it \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix \
  --device=/dev/dri:/dev/dri \
  --env="DISPLAY" \
  --name='ros_image'\
  robotik:kinetic
