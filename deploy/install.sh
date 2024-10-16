# !/bin/bash
TARGET_DIR="/etc/ryankuederle/ros2-app-deployment"
if [ -d "$TARGET_DIR" ]; then
    echo "Target directory already exists. Removing it."
    sudo rm -rf "$TARGET_DIR"
else
    echo "Target directory does not exist."
fi
if [ -f "/usr/share/applications/ros2-app-deployment.desktop" ]; then
    echo "Desktop application already exists."
    sudo rm /usr/share/applications/ros2-app-deployment.desktop
else
    echo "Desktop application does not exist."
fi
sudo cp "$(dirname "$0")/ros2-app-deployment.desktop" "/usr/share/applications"

sudo mkdir -p "$TARGET_DIR"
echo "Created directory $TARGET_DIR"
sudo cp "$(dirname "$0")/network_create.sh" "$TARGET_DIR"
sudo cp "$(dirname "$0")/execute.sh" "$TARGET_DIR"
sudo cp "$(dirname "$0")/ap_icon.png" "$TARGET_DIR"

sudo docker build -t ros2-deployment "$(dirname "$0")/.."

sudo xhost +local:root