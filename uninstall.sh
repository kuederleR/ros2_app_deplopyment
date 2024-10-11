#!/bin/bash
container_name="ros2-deployment"

if [ "$(docker ps -aq -f name=$container_name)" ]; then
    docker rm -f $container_name
    echo "Container $container_name removed."
else
    echo "Container $container_name does not exist."
fi

folder_path="/etc/ryankuederle/ros2-app-deployment"

if [ -d "$folder_path" ]; then
    rm -rf "$folder_path"
    echo "Folder $folder_path removed."
else
    echo "Folder $folder_path does not exist."
fi

desktop_file="/usr/share/applications/ros2-app-deployment.desktop"

if [ -f "$desktop_file" ]; then
    rm "$desktop_file"
    echo "Desktop file $desktop_file removed."
else
    echo "Desktop file $desktop_file does not exist."
fi