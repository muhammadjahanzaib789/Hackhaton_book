#!/usr/bin/env python3
"""
Code Example: Simple ROS 2 Publisher

Problem Solved: Demonstrates how to create a ROS 2 node that publishes messages
to a topic. This is the fundamental building block for sending sensor data,
commands, or any information between nodes in a humanoid robot system.

Assumptions:
- ROS 2 Humble is installed and sourced
- rclpy and std_msgs packages are available
- Running on Linux with Python 3.10+

Failure Modes:
- If ROS 2 not sourced: "ModuleNotFoundError: No module named 'rclpy'"
- If topic name conflicts: Messages may go to wrong subscribers
- If timer too fast: May overwhelm subscribers or network

Input: None (self-generates data)
Output: String messages on /humanoid/status topic at 1Hz

Usage:
    ros2 run my_robot_pkg simple_publisher
    # Or directly:
    python3 simple_publisher.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HumanoidStatusPublisher(Node):
    """
    A simple publisher node for humanoid robot status updates.

    Why a class? ROS 2 uses object-oriented design to encapsulate node state,
    making it easier to manage complex robots with multiple publishers/subscribers.
    """

    def __init__(self):
        # Initialize the node with a unique name
        # This name appears in ros2 node list and must be unique in the graph
        super().__init__('humanoid_status_publisher')

        # Create a publisher for String messages on the /humanoid/status topic
        # Queue size of 10 means up to 10 messages can be buffered if subscriber is slow
        self.publisher_ = self.create_publisher(
            String,              # Message type
            'humanoid/status',   # Topic name (no leading slash = relative)
            10                   # Queue depth
        )

        # Create a timer that calls publish_status every 1 second
        # Why 1Hz? Matches typical status update rate; adjust based on your needs
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_status)

        # Counter for demo purposes - tracks how many messages sent
        self.count = 0

        self.get_logger().info('Humanoid status publisher initialized')

    def publish_status(self):
        """
        Timer callback that publishes a status message.

        In a real humanoid robot, this would aggregate sensor data,
        joint states, and system health into a status message.
        """
        msg = String()
        msg.data = f'Humanoid status update #{self.count}: All systems nominal'

        # Publish the message to all subscribers
        self.publisher_.publish(msg)

        # Log for debugging (visible in terminal and ros2 log)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.count += 1


def main(args=None):
    """
    Main entry point for the publisher node.

    Pattern: Initialize -> Spin -> Shutdown
    This pattern is used in all ROS 2 Python nodes.
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create the node instance
    node = HumanoidStatusPublisher()

    try:
        # Spin the node so callbacks are processed
        # This blocks until the node is shutdown (Ctrl+C)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Clean exit on Ctrl+C
    finally:
        # Clean up: destroy the node and shutdown rclpy
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
