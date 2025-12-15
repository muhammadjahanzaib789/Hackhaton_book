#!/usr/bin/env python3
"""
Code Example: Simple ROS 2 Subscriber

Problem Solved: Demonstrates how to create a ROS 2 node that subscribes to
messages from a topic. This is essential for receiving sensor data, commands,
or status updates in a humanoid robot system.

Assumptions:
- ROS 2 Humble is installed and sourced
- rclpy and std_msgs packages are available
- A publisher is running on the /humanoid/status topic

Failure Modes:
- If no publisher exists: Subscriber waits indefinitely (no error)
- If message type mismatch: "TypeMismatch" error and messages dropped
- If callback takes too long: Messages queue up, may cause latency

Input: String messages from /humanoid/status topic
Output: Logs received messages to console

Usage:
    # Terminal 1: Start publisher first
    python3 simple_publisher.py

    # Terminal 2: Start subscriber
    python3 simple_subscriber.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HumanoidStatusSubscriber(Node):
    """
    A simple subscriber node for humanoid robot status updates.

    Why subscribe? In robotics, different nodes handle different tasks.
    A navigation node might subscribe to sensor data, while a logging node
    subscribes to status updates. This separation of concerns makes
    complex robots manageable.
    """

    def __init__(self):
        super().__init__('humanoid_status_subscriber')

        # Create a subscription to the /humanoid/status topic
        # The callback is invoked whenever a new message arrives
        self.subscription = self.create_subscription(
            String,                    # Message type (must match publisher)
            'humanoid/status',         # Topic name (must match publisher)
            self.status_callback,      # Callback function
            10                         # Queue depth
        )

        # Track message count for statistics
        self.message_count = 0

        self.get_logger().info('Humanoid status subscriber initialized')
        self.get_logger().info('Waiting for messages on /humanoid/status...')

    def status_callback(self, msg: String):
        """
        Callback invoked when a message is received.

        In a real humanoid robot, this callback might:
        - Parse sensor data and trigger safety checks
        - Update an internal state machine
        - Forward data to other nodes for processing

        Keep callbacks fast! Long processing blocks the executor.
        For heavy work, use separate threads or async patterns.
        """
        self.message_count += 1

        # Log the received message
        self.get_logger().info(
            f'Received [{self.message_count}]: "{msg.data}"'
        )

        # Example: React to specific status
        if 'nominal' in msg.data.lower():
            self.get_logger().debug('System healthy - no action needed')
        elif 'error' in msg.data.lower():
            self.get_logger().warn('Error detected in status message!')


def main(args=None):
    """
    Main entry point for the subscriber node.

    Same pattern as publisher: Initialize -> Spin -> Shutdown
    """
    rclpy.init(args=args)

    node = HumanoidStatusSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f'Shutting down. Received {node.message_count} messages total.'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
