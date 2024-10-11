# !/bin/bash
# bash /etc/ryankuederle/ros2-app-deployment/network_create.sh
# docker run -it --rm --net=host --env="DISPLAY" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" ros2-deployment
docker run -it --rm --net=host --ipc=host --pid=host --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" ros2-deployment 
# Maybe install xvfb?