#!/bin/bash

git clone https://github.com/kuederleR/ros2_app_deplopyment.git /tmp/ros2_app_deployment
cd /tmp/ros2_app_deployment
sudo bash deploy/install.sh
cd ..
sudo rm -rf /tmp/ros2_app_deployment