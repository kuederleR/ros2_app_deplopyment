#!/bin/bash
source /opt/ros/humble/setup.bash \
&& source /ros2_app_deployment/install/setup.bash \
&& ros2 run base_pkg keyboard_publisher
