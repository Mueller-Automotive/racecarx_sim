# ROS2 and Gazebo simulation

## ROS2 installation
Ensure you are running an Ubuntu 22 (jammy) based distro (e.g. Mint 21)
```
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

if ! [ -f /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg ]
then
    sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
fi
if ! [ -f /etc/apt/sources.list.d/gazebo-prerelease.list ]
then
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-prerelease $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-prerelease.list > /dev/null
fi

sudo apt update
sudo apt upgrade

sudo apt-get install --no-install-recommends -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    gz-garden \
    ros-humble-actuator-msgs \
    ros-humble-ros-gzgarden-bridge \
    ros-humble-ros-gzgarden-image \
    ros-humble-ros-gzgarden-sim \

echo -e "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash
```

## Clone, build and launch
For the time being, actuator message bridging in the `ros-humble-ros-gz` package appears to be broken, so we clone locally and build from source.
```
mkdir mueller-auto-sim
cd mueller-auto-sim
git clone git@github.com:Mueller-Automotive/mueller_auto.git
git clone git@github.com:gazebosim/ros_gz.git

colcon build --symlink-install --parallel-workers 8

source install/setup.bash
ros2 launch mueller_auto launch_sim.launch.py
```

## Control
```
gz topic --topic /actuators --msgtype gz.msgs.Actuators -p"
position: [-0.3]
velocity: [5]
"
```
Where `position` is the steering angle in radians and `velocity` is the revolution speed of the wheels in radians per second