"""
Full bringup launch file for TurtleBot3 Burger room exploration.

Launches the complete stack in one command:
  1. TurtleBot3 bringup  (LiDAR, motors, odometry, URDF TF)
  2. SLAM Toolbox async  (subscribes /scan, publishes /map + map→odom TF)
  3. Nav2 stack          (planner, controller, costmaps, behaviors)
  4. Exploration node    (the algorithm orchestrator)

Usage:
  export TURTLEBOT3_MODEL=burger
  ros2 launch room_exploration bringup.launch.py strategy:=bfs
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('room_exploration')
    config_dir = os.path.join(pkg_dir, 'config')
    tb3_bringup_dir = get_package_share_directory('turtlebot3_bringup')

    # ── Launch arguments ────────────────────────────────────────────
    strategy_arg = DeclareLaunchArgument(
        'strategy',
        default_value='frontier',
        description='Exploration algorithm: frontier, bfs, information_gain, rrt, voronoi',
    )

    strategy = LaunchConfiguration('strategy')

    # Config file paths
    exploration_config = os.path.join(config_dir, 'exploration.yaml')
    mapper_config = os.path.join(config_dir, 'mapper_params.yaml')
    nav2_config = os.path.join(config_dir, 'nav2_params.yaml')

    # ── 1. TurtleBot3 bringup ───────────────────────────────────────
    tb3_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_bringup_dir, 'launch', 'robot.launch.py')
        ),
    )

    # ── 2. SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[mapper_config],
        arguments=['--ros-args', '--log-level', 'slam_toolbox:=INFO'],
    )

    # ── 3. Nav2 stack ───────────────────────────────────────────────
    nav2_nodes = GroupAction(
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[nav2_config],
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[nav2_config],
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[nav2_config],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[nav2_config],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[nav2_config],
            ),
        ]
    )

    # ── 4. Exploration node ─────────────────────────────────────────
    exploration_node = Node(
        package='room_exploration',
        executable='main_node',
        name='exploration_node',
        output='screen',
        parameters=[
            exploration_config,
            {'active_strategy': strategy},
        ],
    )

    return LaunchDescription([
        strategy_arg,
        tb3_bringup,
        TimerAction(period=14.0, actions=[slam_node]),
        TimerAction(period=18.0, actions=[nav2_nodes]),
        TimerAction(period=22.0, actions=[exploration_node]),
    ])
