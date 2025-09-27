#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    # Launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/uwb/raw_data',
        description='Input topic name for UWB raw data'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic', 
        default_value='/uwb/coordinates',
        description='Output topic name for parsed coordinates'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='uwb_frame',
        description='Frame ID for coordinate data'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error, fatal)'
    )
    
    # UWB Parser Node
    uwb_parser_node = Node(
        package='uwb_parser',
        executable='uwb_coordinate_parser',
        name='uwb_coordinate_parser',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'frame_id': LaunchConfiguration('frame_id'),
        }]
    )
    
    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        frame_id_arg,
        log_level_arg,
        uwb_parser_node
    ])