# ROS2 and Gazebo simulation

## ROS2 installation
Ensure you are running an Ubuntu 22 (jammy) based distro (e.g. Mint 21)
```
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade

sudo apt install ros-humble-desktop ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs

echo -e "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash
```

## Clone, build and launch

```
mkdir mueller_auto
cd mueller_auto
git clone git@github.com:Mueller-Automotive/mueller-auto-sim.git

colcon build --symlink-install

source install/setup.bash
ros2 launch mueller_auto launch_sim.launch.py
```