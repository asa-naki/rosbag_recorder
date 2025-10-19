#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Simple launch file for testing with minimal configuration
    """
    
    # ROS2 Rosbag Recorder Node with minimal settings
    rosbag_recorder_node = Node(
        package='rosbag_recorder',
        executable='rosbag_recorder_node',
        name='rosbag_recorder',
        output='screen',
        parameters=[
            {
                'output_directory': '/tmp/rosbag_test',
                'split_duration_minutes': 1,
                'bag_name_prefix': 'test_recording',
                'storage_id': 'sqlite3',
                'serialization_format': 'cdr',
                'max_bagfile_size': 0,
                'max_cache_size': 104857600,
                'include_hidden_topics': False,
                'record_services': False,
            }
        ],
        remappings=[
            ('~/start_recording', '/rosbag/start_recording'),
            ('~/stop_recording', '/rosbag/stop_recording'),
        ]
    )

    return LaunchDescription([
        rosbag_recorder_node
    ])
