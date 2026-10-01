#!/usr/bin/env python3
"""
Launch-File für tb3_maze_navigation

Startet wahlweise Wall-Following oder Pledge.
Aufruf:
    ros2 launch tb3_maze_navigation maze.launch.py algorithm:=wall_following
    ros2 launch tb3_maze_navigation maze.launch.py algorithm:=pledge
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    algorithm_arg = DeclareLaunchArgument(
        'algorithm',
        default_value='pledge',
        description='Algorithmus: "wall_following" oder "pledge"'
    )

    algorithm = LaunchConfiguration('algorithm')

    nav_node = Node(
        package='tb3_maze_navigation',
        executable=algorithm,
        name='maze_navigator',
        output='screen',
        parameters=[{
            # Parameter können hier überschrieben werden wenn du
            # sie später als ROS-Parameter statt Konstanten implementierst
        }]
    )

    return LaunchDescription([
        algorithm_arg,
        nav_node,
    ])
