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

    # steering bridge
    cmd_vel_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                          name='cmd_vel_bridge',
                          output='screen',
                          parameters=[{
                              'use_sim_time': True
                          }],
                          arguments=[
                              '/model/racecarx/cmd_vel' + '@geometry_msgs/msg/Twist' + ']gz.msgs.Twist',
                          ])

    # actuators bridge
    actuators_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                          name='actuators_bridge',
                          output='screen',
                          parameters=[{
                              'use_sim_time': True
                          }],
                          arguments=[
                              '/actuators' + '@actuator_msgs/msg/Actuators' + ']gz.msgs.Actuators',
                          ])
    
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'racecarx-depot',
            '-name', 'racecarx',
            '-x', '8.7566537354533125e-11',
            '-y', '2.0000000001634253',
            '-z', '0.3250071890159123',
            '-file', PathJoinSubstitution(
                [get_package_share_directory(package_name),
                'description',
                'car.sdf'])
        ],
        output='screen'
    )
    # 8.7566537354533125e-11 2.0000000001634253 0.3250071890159123 -5.0518222677781699e-10 3.1007408737803522e-10 -5.1850145662276096e-12
    # Launch them all!
    return LaunchDescription([
        #rsp,
        #camera,
        actuators_bridge,
        cmd_vel_bridge,
        gz_sim,
        spawn_robot,
    ])
