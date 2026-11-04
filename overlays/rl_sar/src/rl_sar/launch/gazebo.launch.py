# Modified by Dubai Future Labs: DDDMR simulation integration.
# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rname = LaunchConfiguration("rname")

    dome_lidar_arguments = {
        "dome_lidar_enabled": ("true", "Enable the dome LiDAR on supported robot descriptions"),
        "dome_lidar_backend": ("gpu", "Dome LiDAR backend (gpu or cpu)"),
        "dome_lidar_gpu_resolution": ("2048", "Pixels per GPU cube face edge; output sample count is unchanged"),
        "dome_lidar_xyz": ("0.32 0 0.10", "LiDAR optical origin relative to the robot body (m)"),
        "dome_lidar_rpy": ("0 0.7853981633974483 0", "LiDAR mounting roll, pitch, yaw (rad)"),
        "dome_lidar_horizontal_samples": ("900", "Number of azimuth samples per frame"),
        "dome_lidar_vertical_samples": ("128", "Number of elevation samples per frame"),
        "dome_lidar_update_rate": ("10.0", "LiDAR frame rate (Hz)"),
        "dome_lidar_min_range": ("0.1", "LiDAR minimum range (m)"),
        "dome_lidar_max_range": ("60.0", "LiDAR maximum range (m)"),
        "dome_lidar_topic": ("/dome_lidar/points", "LiDAR PointCloud2 topic"),
    }

    wname = "complex_terrain_v6"
    robot_name = ParameterValue(Command(["echo -n ", rname]), value_type=str)
    ros_namespace = ParameterValue(Command(["echo -n ", "/", rname, "_gazebo"]), value_type=str)
    gazebo_model_name = ParameterValue(Command(["echo -n ", rname, "_gazebo"]), value_type=str)

    xacro_command = [
        "xacro ",
        Command(["echo -n ", Command(["ros2 pkg prefix ", rname, "_description"])]),
        "/share/", rname, "_description/xacro/robot.xacro",
    ]
    for name in dome_lidar_arguments:
        # Quote values so xyz/rpy remain a single xacro mapping argument.
        xacro_command.extend([" ", name, ":='", LaunchConfiguration(name), "'"])
    robot_description = ParameterValue(Command(xacro_command), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            # "verbose": "true",
            # "pause": "true",  # Not Available
            "world": os.path.join(get_package_share_directory("rl_sar"), "worlds", wname + ".world"),
        }.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "/robot_description",
            "-entity", "robot_model",
            "-z", "1.0",
        ],
        output="screen",
    )

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable='spawner.py' if os.environ.get('ROS_DISTRO', '') == 'foxy' else 'spawner',
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    robot_joint_controller_node = Node(
        package="controller_manager",
        executable='spawner.py' if os.environ.get('ROS_DISTRO', '') == 'foxy' else 'spawner',
        arguments=["robot_joint_controller"],
        output="screen",
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'deadzone': 0.1,
            'autorepeat_rate': 0.0,
        }],
    )

    param_node = Node(
        package="demo_nodes_cpp",
        executable="parameter_blackboard",
        name="param_node",
        parameters=[{
            "robot_name": robot_name,
            "gazebo_model_name": gazebo_model_name,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "rname",
            description="Robot name (e.g., a1, go2)",
            default_value=TextSubstitution(text="lite3"),
        ),
        *[
            DeclareLaunchArgument(name, default_value=default, description=description)
            for name, (default, description) in dome_lidar_arguments.items()
        ],
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_node,
        # robot_joint_controller_node,  # Spawn in rl_sim.cpp
        joy_node,
        param_node,
    ])
