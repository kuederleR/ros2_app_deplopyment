#!/bin/bash

if ! command -v docker &> /dev/null
then
    echo "Docker is not installed. Installing Docker..."
    sudo apt-get update
    sudo apt-get install -y docker.io
else
    echo "Docker is already installed."
fi

sudo usermod -aG docker $USER

git clone https://github.com/kuederleR/ros2_app_deplopyment.git /tmp/ros2-deployment
cd /tmp/ros2_app_deployment
sudo bash deploy/install.sh
cd ..
sudo rm -rf /tmp/ros2-deployment
