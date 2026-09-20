import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    upstream_share = get_package_share_directory('saye_bringup')
    plugin_share = get_package_share_directory('ackermann_plugin_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Open the upstream simulation RViz.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                upstream_share, 'launch', 'saye_spawn.launch.py')),
            launch_arguments={'rviz': LaunchConfiguration('rviz')}.items(),
        ),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='ackermann_joint_state_bridge', output='screen',
            parameters=[{
                'use_sim_time': True,
                'config_file': os.path.join(
                    plugin_share, 'config', 'joint_state_bridge.yaml'),
            }],
        ),
        Node(
            package='utils', executable='joint_to_ackermann',
            name='joint_to_ackermann_node', output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
