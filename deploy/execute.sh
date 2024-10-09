# !/bin/bash
# bash /etc/ryankuederle/ros2-app-deployment/network_create.sh
docker run -it --rm --net=host --env="DISPLAY" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" ros2-deployment
