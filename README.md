# ROS2 and Gazebo simulation

## Clone, build and launch

```
mkdir mueller_auto
cd mueller_auto
git clone git@github.com:Mueller-Automotive/mueller_auto_sim.git

colcon build --symlink-install

source install/setup.bash
ros2 launch mueller_auto launch_sim.launch.py
```