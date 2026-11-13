# Modified by Dubai Future Labs: Gazebo Harmonic integration.
# SPDX-License-Identifier: Apache-2.0
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sensor = get_package_share_directory('robot_sensors_description')
    robot = get_package_share_directory('lite3_description')
    package = get_package_share_directory('rl_sar')
    parameters = {
        'dome_lidar_enabled': 'true', 'dome_lidar_backend': 'gpu',
        'dome_lidar_gpu_resolution': '2048', 'dome_lidar_xyz': '0.32 0 0.10',
        'dome_lidar_rpy': '0 0.7853981633974483 0',
        'dome_lidar_horizontal_samples': '900', 'dome_lidar_vertical_samples': '128',
        'dome_lidar_update_rate': '10.0', 'dome_lidar_min_range': '0.1',
        'dome_lidar_max_range': '60.0', 'dome_lidar_topic': '/dome_lidar/points',
    }
    command = ['xacro ', os.path.join(robot, 'xacro', 'robot.xacro')]
    for name in parameters:
        command.extend([' ', name, ":='", LaunchConfiguration(name), "'"])
    description = ParameterValue(Command(command), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('rname', default_value='lite3'),
        *[DeclareLaunchArgument(name, default_value=value) for name,value in parameters.items()],
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(robot)),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(sensor)),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': True}], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': '-r -v 3 ' + os.path.join(package,'worlds','complex_terrain_v6.world')}.items()),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-topic','/robot_description','-name','robot_model','-z','1.0'], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/world/complex_terrain_v6/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                        '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                        '/simulation/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                        '/simulation/reset_robot@ros_gz_interfaces/srv/ControlWorld',
                        '/world/complex_terrain_v6/control@ros_gz_interfaces/srv/ControlWorld'],
             remappings=[('/world/complex_terrain_v6/clock', '/clock')],
             parameters=[{'qos_overrides./clock.publisher.durability':'volatile'}], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster','--controller-manager-timeout','120'], output='screen'),
        Node(package='joy', executable='joy_node', parameters=[{'deadzone':0.1,'autorepeat_rate':0.0}]),
        Node(package='demo_nodes_cpp', executable='parameter_blackboard', name='param_node',
             parameters=[{'robot_name':'lite3','gazebo_model_name':'robot_model'}]),
    ])
