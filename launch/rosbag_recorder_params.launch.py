#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Check ROS distro for service recording support
    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    default_record_services = 'true' if ros_distro == 'jazzy' else 'false'
    default_services = '[]' if ros_distro == 'humble' else '["/get_map"]'
    # Declare launch arguments for all parameters
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
    
    storage_id_arg = DeclareLaunchArgument(
        'storage_id',
        default_value='sqlite3',
        description='Storage format (sqlite3, mcap)'
    )
    
    topics_arg = DeclareLaunchArgument(
        'topics',
        default_value='["/camera/image_raw", "/lidar/points", "/imu/data", "/tf", "/tf_static"]',
        description='List of topics to record'
    )
    
    topic_regex_patterns_arg = DeclareLaunchArgument(
        'topic_regex_patterns',
        default_value='["/sensor_.*", "/robot/.*", ".*_status$"]',
        description='List of regex patterns for topic matching'
    )
    
    services_arg = DeclareLaunchArgument(
        'services',
        default_value=default_services,
        description=f'List of services to record ({"supported" if ros_distro == "jazzy" else "not supported"} in {ros_distro})'
    )
    
    record_services_arg = DeclareLaunchArgument(
        'record_services',
        default_value=default_record_services,
        description=f'Whether to record services ({"supported" if ros_distro == "jazzy" else "not supported"} in {ros_distro})'
    )

    # ROS2 Rosbag Recorder Node
    rosbag_recorder_node = Node(
        package='rosbag_recorder',
        executable='rosbag_recorder_node',
        name='rosbag_recorder',
        output='screen',
        parameters=[
            {
                'output_directory': LaunchConfiguration('output_directory'),
                'split_duration_minutes': LaunchConfiguration('split_duration_minutes'),
                'bag_name_prefix': LaunchConfiguration('bag_name_prefix'),
                'storage_id': LaunchConfiguration('storage_id'),
                'serialization_format': 'cdr',
                'max_bagfile_size': 0,
                'max_cache_size': 104857600,  # 100MB
                'topics': LaunchConfiguration('topics'),
                'topic_regex_patterns': LaunchConfiguration('topic_regex_patterns'),
                'services': LaunchConfiguration('services'),
                'include_hidden_topics': False,
                'record_services': LaunchConfiguration('record_services'),
            }
        ],
        # Remap service names for easier access
        remappings=[
            ('~/start_recording', '/rosbag/start_recording'),
            ('~/stop_recording', '/rosbag/stop_recording'),
        ]
    )

    return LaunchDescription([
        output_directory_arg,
        split_duration_arg,
        bag_name_prefix_arg,
        storage_id_arg,
        topics_arg,
        topic_regex_patterns_arg,
        services_arg,
        record_services_arg,
        rosbag_recorder_node
    ])
