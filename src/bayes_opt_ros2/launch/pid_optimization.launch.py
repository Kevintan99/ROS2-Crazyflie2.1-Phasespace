#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('bayes_opt_ros2')
    
    # Launch arguments
    config_file = LaunchConfiguration('config_file')
    
    # Declare launch arguments
    declare_config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'pid_payload_optimization.yaml'),
        description='Path to PID optimization configuration file'
    )
    
    # Bayesian optimization node
    bayes_opt_node = Node(
        package='bayes_opt_ros2',
        executable='bayesian_opt_node.py',
        name='bayes_opt_node',
        output='screen',
        parameters=[config_file],
        prefix=['xterm -e']
    )
    
    # PID experiment client (start with delay to ensure optimization node is ready)
    pid_client_node = Node(
        package='bayes_opt_ros2',
        executable='pid_payload_experiment_client.py',
        name='pid_experiment_client',
        output='screen',
        prefix=['xterm -e']
    )
    
    # Delay the client start
    delayed_client = TimerAction(
        period=5.0,  # Wait 5 seconds
        actions=[pid_client_node]
    )
    
    return LaunchDescription([
        declare_config_arg,
        bayes_opt_node,
        delayed_client
    ]) 