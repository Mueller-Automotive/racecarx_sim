import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('world', default_value='depot',
                          description='GZ World'),
    DeclareLaunchArgument('robot_name', default_value='racercarx',
                          description='Robot name')
]


def generate_launch_description():
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    
    # Make sure package name is correct
    package_name='mueller_auto' # <--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    camera = rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','camera.launch.py'
                )])
    )

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim') #checked
    gz_sim_launch = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py']) #checked

    # Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('gz_args', [PathJoinSubstitution([get_package_share_directory(package_name), 'worlds', 'depot.sdf'])])
        ]
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'racecarx-depot',
            '-name', 'racecarx',
            '-file', PathJoinSubstitution(
                [get_package_share_directory(package_name),
                'description',
                'car.sdf'])
        ],
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        #rsp,
        #camera,
        gz_sim,
        spawn_robot,
    ])
