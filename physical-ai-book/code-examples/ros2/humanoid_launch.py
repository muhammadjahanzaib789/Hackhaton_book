#!/usr/bin/env python3
"""
Code Example: ROS 2 Launch File for Humanoid Robot

Problem Solved: Demonstrates how to create a launch file that starts multiple
ROS 2 nodes together. Launch files are essential for humanoid robots with
dozens of nodes (sensors, controllers, planners) that must start in coordination.

Assumptions:
- ROS 2 Humble is installed and sourced
- launch and launch_ros packages are available
- The nodes referenced exist in your workspace

Failure Modes:
- If node package not found: "PackageNotFoundError"
- If node executable not found: "ExecutableNotFoundError"
- If parameters invalid: Node may crash on startup
- If nodes depend on each other: Use lifecycle nodes or explicit dependencies

Input: None (configuration embedded or via parameters)
Output: Multiple nodes running in coordinated fashion

Usage:
    ros2 launch my_robot_pkg humanoid_launch.py

    # With parameters:
    ros2 launch my_robot_pkg humanoid_launch.py robot_name:=atlas sim_mode:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    """
    Generate the launch description for humanoid robot nodes.

    Launch files in ROS 2 are Python scripts that return a LaunchDescription.
    This gives you full programmatic control over:
    - Which nodes to start
    - Parameters and remappings
    - Conditional logic
    - Grouping and namespacing
    """

    # Declare launch arguments (can be overridden from command line)
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='humanoid',
        description='Name of the robot (used for namespacing)'
    )

    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='true',
        description='Whether to run in simulation mode'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )

    # Get launch configuration values
    robot_name = LaunchConfiguration('robot_name')
    sim_mode = LaunchConfiguration('sim_mode')
    log_level = LaunchConfiguration('log_level')

    # Status publisher node
    status_publisher = Node(
        package='my_robot_pkg',  # Replace with your package name
        executable='simple_publisher',
        name='status_publisher',
        output='screen',
        parameters=[{
            'publish_rate': 1.0,
            'robot_name': robot_name,
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # Status subscriber node (for monitoring)
    status_subscriber = Node(
        package='my_robot_pkg',
        executable='simple_subscriber',
        name='status_monitor',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )

    # Motor control service (only in non-sim mode for safety)
    motor_service = Node(
        package='my_robot_pkg',
        executable='service_server',
        name='motor_controller',
        output='screen',
        parameters=[{
            'safety_enabled': True,
        }],
        condition=IfCondition(
            PythonExpression(["'", sim_mode, "' == 'false'"])
        ),
    )

    # Simulation motor service (mock version for sim mode)
    sim_motor_service = Node(
        package='my_robot_pkg',
        executable='service_server',
        name='sim_motor_controller',
        output='screen',
        parameters=[{
            'safety_enabled': False,  # Simulation doesn't need hardware safety
            'simulated': True,
        }],
        condition=IfCondition(sim_mode),
    )

    # Group nodes under robot namespace
    # All topics/services will be prefixed with /{robot_name}/
    robot_group = GroupAction([
        PushRosNamespace(robot_name),
        status_publisher,
        status_subscriber,
        motor_service,
        sim_motor_service,
    ])

    # Log startup information
    startup_log = LogInfo(
        msg=['Starting humanoid robot: ', robot_name, ' (sim_mode=', sim_mode, ')']
    )

    return LaunchDescription([
        # Declare arguments first
        robot_name_arg,
        sim_mode_arg,
        log_level_arg,

        # Log startup
        startup_log,

        # Launch robot nodes
        robot_group,
    ])


# Alternative: Minimal launch file structure for quick testing
"""
# Minimal version - put this in a separate file like minimal_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_pkg',
            executable='simple_publisher',
            name='publisher',
        ),
        Node(
            package='my_robot_pkg',
            executable='simple_subscriber',
            name='subscriber',
        ),
    ])
"""
