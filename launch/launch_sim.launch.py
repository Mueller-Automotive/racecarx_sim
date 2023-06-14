import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    
    # Make sure package name is correct
    package_name='racecarx-sim' # <--- CHANGE ME

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_sim_launch = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    # Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('gz_args', [
                PathJoinSubstitution([get_package_share_directory(package_name), 'worlds', 'depot.sdf']),
                 ' -r'])
        ]
    )

    # Bridge Actuators messages between ROS and Gazebo
    actuators_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                            name='actuators_bridge',
                            output='screen',
                            parameters=[{
                                'use_sim_time': True
                            }],
                            arguments=[
                                '/actuators' + '@actuator_msgs/msg/Actuators' + ']gz.msgs.Actuators',
                            ])
    
    # Spawn an entity of the car in the world we started
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'racecarx-depot',
            '-name', 'racecarx',
            '-x', '0',
            '-y', '2',
            '-z', '0.35',
            '-file', PathJoinSubstitution(
                [get_package_share_directory(package_name),
                'description',
                'car.sdf'])
        ],
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        actuators_bridge,
        gz_sim,
        spawn_robot,
    ])
