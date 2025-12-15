#!/usr/bin/env python3
"""
RSAEM Robot Web Interface Launch File

Launches:
- Robot bringup (motors, sensors, TF)
- Jetson camera node
- ROSBridge WebSocket server (port 9090)
- Web Video Server (port 8080)

Static files served by nginx on port 80:
- Install: sudo cp config/nginx/rssaem.conf /etc/nginx/sites-available/
- Enable: sudo ln -sf /etc/nginx/sites-available/rssaem.conf /etc/nginx/sites-enabled/
- Restart: sudo systemctl restart nginx

Access: http://<robot_ip>/
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    rssaem_bringup_dir = get_package_share_directory('rssaem_bringup')
    rssaem_ai_dir = get_package_share_directory('rssaem_ai')

    return LaunchDescription([
        # Include robot bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rssaem_bringup_dir, 'launch', 'rssaem.launch.py')
            )
        ),

        # Include Jetson camera node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rssaem_ai_dir, 'launch', 'camera.launch.py')
            ),
            launch_arguments={
                'width': '640',
                'height': '480',
                'fps': '15',
            }.items(),
        ),

        # ROSBridge WebSocket Server (proxied via nginx /rosbridge)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'port': 9090,
                'address': '127.0.0.1',  # Local only, nginx proxies
                'retry_startup_delay': 5.0,
            }],
            output='screen'
        ),

        # Web Video Server (proxied via nginx /stream)
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            parameters=[{
                'port': 8080,
                'address': '127.0.0.1',  # Local only, nginx proxies
                'default_stream_type': 'mjpeg',
                'quality': 50,
            }],
            output='screen'
        ),
    ])
