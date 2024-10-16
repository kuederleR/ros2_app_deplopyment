FROM ros:humble
ARG USERNAME=$(whoami)
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    # && apt-get install -y \
    # && apt install ros-humble-rviz2 -y \
    # && apt-get install ros-humble-demo-nodes-py -y \
    && apt-get install -y \
    libxcb-xinerama0 \
    libxcb1 \
    libxcb-xinput0 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-render-util0 \
    && rm -rf /var/lib/apt/lists/* \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip\
    && pip install setuptools==58.2.0

ENV SHELL /bin/bash

COPY . /ros2_app_deployment

# ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=:0

RUN rosdep update \
    && cd /ros2_app_deployment \
    && rosdep install --from-paths src --ignore-src -r -y \
    && colcon build


# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
# USER $USERNAME
# CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_app_deployment/install/setup.bash && /bin/bash"]
COPY entrypoint.sh /ros2_app_deployment/entrypoint.sh
# RUN chmod +x /ros2_app_deployment/entrypoint.sh
# ENTRYPOINT ["/ros2_app_deployment/entrypoint.sh"]


# Run export RMW_IMPLEMENTATION=rmw_fastrtps_cpp -- Don't need to do this anymore

# docker run -it --rm --net=host --ipc=host --pid=host --env="DISPLAY" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" ros2-deployment 


# docker run -it --rm --net=host --ipc=host --pid=host --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" ros2-deployment 