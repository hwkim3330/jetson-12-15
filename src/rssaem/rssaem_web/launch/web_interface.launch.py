#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    rssaem_bringup_dir = get_package_share_directory('rssaem_bringup')
    rssaem_web_dir = get_package_share_directory('rssaem_web')

    # Web server directory
    www_dir = os.path.join(rssaem_web_dir, 'www')

    return LaunchDescription([
        # Include robot bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rssaem_bringup_dir, 'launch', 'rssaem.launch.py')
            )
        ),

        # ROSBridge WebSocket Server
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'port': 9090,
                'address': '',
                'retry_startup_delay': 5.0,
            }],
            output='screen'
        ),

        # Web Video Server (for camera streaming)
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            parameters=[{
                'port': 8080,
                'address': '0.0.0.0',
            }],
            output='screen'
        ),

        # Python HTTP Server for web interface
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '8888', '--directory', www_dir],
            output='screen'
        ),
    ])
