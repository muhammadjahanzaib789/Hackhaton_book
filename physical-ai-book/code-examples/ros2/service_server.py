#!/usr/bin/env python3
"""
Code Example: ROS 2 Service Server

Problem Solved: Demonstrates how to create a ROS 2 service server that responds
to synchronous requests. Services are used for request-response interactions
like querying robot state, triggering calibration, or requesting specific actions.

Assumptions:
- ROS 2 Humble is installed and sourced
- rclpy and std_srvs packages are available
- Service name does not conflict with other services

Failure Modes:
- If service already exists: "ServiceNameNotUnique" error
- If callback throws exception: Client receives error response
- If callback blocks too long: Client times out, other requests queue up
- If server dies: Pending requests fail with "ServiceNotAvailable"

Input: SetBool request (bool data)
Output: SetBool response (bool success, string message)

Usage:
    # Terminal 1: Start service server
    python3 service_server.py

    # Terminal 2: Call service
    ros2 service call /humanoid/enable_motors std_srvs/srv/SetBool "{data: true}"
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class HumanoidMotorService(Node):
    """
    A service server that enables/disables humanoid robot motors.

    Why services vs topics?
    - Topics: Continuous data streams (sensors, status)
    - Services: One-time request-response (commands, queries)

    Use services when you need confirmation that an action completed.
    """

    def __init__(self):
        super().__init__('humanoid_motor_service')

        # Track motor state (in real robot, this would interface with hardware)
        self.motors_enabled = False

        # Create the service server
        # When a client calls this service, enable_motors_callback is invoked
        self.srv = self.create_service(
            SetBool,                      # Service type
            'humanoid/enable_motors',     # Service name
            self.enable_motors_callback   # Callback function
        )

        self.get_logger().info('Motor control service ready')
        self.get_logger().info('Call with: ros2 service call /humanoid/enable_motors std_srvs/srv/SetBool "{data: true}"')

    def enable_motors_callback(self, request: SetBool.Request, response: SetBool.Response):
        """
        Service callback that handles motor enable/disable requests.

        In a real humanoid robot, this would:
        - Verify it's safe to enable motors (no faults, proper position)
        - Send commands to motor controllers
        - Wait for confirmation from hardware
        - Return success/failure with details

        Parameters:
            request: Contains 'data' bool (True=enable, False=disable)
            response: Must populate 'success' bool and 'message' string

        Returns:
            The populated response object
        """
        if request.data:
            # Enable motors
            if self.motors_enabled:
                response.success = True
                response.message = 'Motors already enabled'
            else:
                # Simulate enabling motors (real code would talk to hardware)
                self.motors_enabled = True
                response.success = True
                response.message = 'Motors enabled successfully'
                self.get_logger().info('Motors ENABLED')
        else:
            # Disable motors
            if not self.motors_enabled:
                response.success = True
                response.message = 'Motors already disabled'
            else:
                self.motors_enabled = False
                response.success = True
                response.message = 'Motors disabled successfully'
                self.get_logger().info('Motors DISABLED')

        self.get_logger().info(f'Service response: {response.message}')
        return response


def main(args=None):
    """
    Main entry point for the service server.

    Service servers use the same spin pattern as publishers/subscribers.
    The spin call processes incoming service requests.
    """
    rclpy.init(args=args)

    node = HumanoidMotorService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down motor control service')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
