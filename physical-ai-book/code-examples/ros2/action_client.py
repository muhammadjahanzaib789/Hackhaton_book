#!/usr/bin/env python3
"""
Code Example: ROS 2 Action Client

Problem Solved: Demonstrates how to create a ROS 2 action client for long-running
tasks with progress feedback. Actions are ideal for robot behaviors like walking,
manipulation, or navigation that take time and can be monitored or cancelled.

Assumptions:
- ROS 2 Humble is installed and sourced
- rclpy and action_tutorials_interfaces packages are available
- An action server is running (use this with action_server.py or Nav2)

Failure Modes:
- If server not available: Client waits/times out based on wait_for_server
- If goal rejected: on_goal_response callback shows rejection
- If server dies mid-execution: Goal status becomes ABORTED
- If network issues: Feedback stops, eventual timeout

Input: Goal specifying number of steps in Fibonacci sequence
Output: Progress feedback and final result

Usage:
    # This example uses Fibonacci action from tutorials
    # In humanoid robots, replace with navigation or manipulation actions

    # Terminal 1: Start action server (if using Fibonacci example)
    ros2 run action_tutorials_py fibonacci_action_server

    # Terminal 2: Run this client
    python3 action_client.py
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# Using Fibonacci as a stand-in for robot actions
# In real humanoid code, use: NavigateToPose, FollowJointTrajectory, etc.
from action_tutorials_interfaces.action import Fibonacci


class HumanoidActionClient(Node):
    """
    An action client for long-running humanoid robot tasks.

    Why actions vs services?
    - Services: Quick request-response (milliseconds)
    - Actions: Long-running with feedback (seconds to minutes)

    Actions provide:
    - Goal submission with acceptance/rejection
    - Progress feedback during execution
    - Ability to cancel in-progress goals
    - Final result when complete

    This pattern is used for:
    - Walking to a destination (Nav2)
    - Arm manipulation sequences
    - Full-body motion planning
    """

    def __init__(self):
        super().__init__('humanoid_action_client')

        # Create action client
        # The action type and name must match the server
        self._action_client = ActionClient(
            self,
            Fibonacci,           # Action type
            'fibonacci'          # Action name
        )

        self.get_logger().info('Action client initialized')

    def send_goal(self, order: int):
        """
        Send a goal to the action server and wait for result.

        In a humanoid robot, this might be:
        - send_goal(target_pose) for navigation
        - send_goal(trajectory) for arm motion
        - send_goal(gait_params) for walking

        Parameters:
            order: For Fibonacci demo, specifies sequence length
        """
        self.get_logger().info(f'Sending goal: compute Fibonacci sequence of order {order}')

        # Create the goal message
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        # Wait for action server to be available
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server available!')

        # Send goal asynchronously with feedback callback
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Add callback for when goal response is received
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Called when action server accepts/rejects the goal.

        In humanoid robots, rejection might happen if:
        - Robot is busy with another goal
        - Goal is physically impossible (unreachable pose)
        - Safety constraints violated
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Goal REJECTED by action server')
            return

        self.get_logger().info('Goal ACCEPTED, waiting for result...')

        # Request the result asynchronously
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Called periodically during goal execution with progress updates.

        In humanoid robots, feedback might include:
        - Current position vs target position
        - Percentage complete
        - Estimated time remaining
        - Current state (walking, turning, reaching, etc.)
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: partial_sequence = {feedback.partial_sequence}')

    def result_callback(self, future):
        """
        Called when action completes (success, failure, or cancelled).

        In humanoid robots, you'd check the result and:
        - Update state machine
        - Trigger next action in sequence
        - Handle failures gracefully
        """
        result = future.result().result
        self.get_logger().info(f'Result: sequence = {result.sequence}')
        self.get_logger().info('Action complete!')

        # Shutdown after receiving result (demo only)
        rclpy.shutdown()


def main(args=None):
    """
    Main entry point demonstrating action client usage.
    """
    rclpy.init(args=args)

    client = HumanoidActionClient()

    # Send a goal (compute Fibonacci sequence of order 10)
    # In a real robot: client.send_goal(target_pose)
    client.send_goal(order=10)

    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        client.get_logger().info('Cancelled by user')
    finally:
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
