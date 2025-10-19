#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Check ROS distro and set default config file
    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    default_config_file = f'recorder_config_{ros_distro}.yaml' if ros_distro == 'humble' else 'recorder_config.yaml'
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description=f'Configuration file name for {ros_distro} (should be in config directory)'
    )
    
    output_directory_arg = DeclareLaunchArgument(
        'output_directory',
        default_value='/media/ssd/rosbag_recordings',
        description='Directory to save rosbag files'
    )
    
    split_duration_arg = DeclareLaunchArgument(
        'split_duration_minutes',
        default_value='1',
        description='Duration in minutes before splitting bag files'
    )
    
    bag_name_prefix_arg = DeclareLaunchArgument(
        'bag_name_prefix',
        default_value='recording',
        description='Prefix for bag file names'
    )

    # Get the path to the config file
    config_file_path = PathJoinSubstitution([
        FindPackageShare('rosbag_recorder'),
        'config',
        LaunchConfiguration('config_file')
    ])

    # ROS2 Rosbag Recorder Node
    rosbag_recorder_node = Node(
        package='rosbag_recorder',
        executable='rosbag_recorder_node',
        name='rosbag_recorder',
        output='screen',
        parameters=[
            {
                'config_file': config_file_path,
                'output_directory': LaunchConfiguration('output_directory'),
                'split_duration_minutes': LaunchConfiguration('split_duration_minutes'),
                'bag_name_prefix': LaunchConfiguration('bag_name_prefix'),
            }
        ],
        # Remap service names for easier access
        remappings=[
            ('~/start_recording', '/rosbag/start_recording'),
            ('~/stop_recording', '/rosbag/stop_recording'),
        ]
    )

    return LaunchDescription([
        config_file_arg,
        output_directory_arg,
        split_duration_arg,
        bag_name_prefix_arg,
        rosbag_recorder_node
    ])
