#!/usr/bin/env python3
"""pid_controller_node.py — ROS2 wrapper around pid_model.PIDController.

Role in the architecture (same as the original):
    Publishes : /cmd_vel        (geometry_msgs/Twist)
    Subscribes: /line_sensor    (std_msgs/Float32)

Supports live parameter tuning via `ros2 param set /pid_controller_node Kp 0.9`
without restarting the node.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32

from line_follower_az.pid_model import PIDController


class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        self.declare_parameter('Kp', 0.8)
        self.declare_parameter('Ki', 0.02)
        self.declare_parameter('Kd', 0.15)
        self.declare_parameter('base_linear_speed', 0.12)
        self.declare_parameter('min_linear_speed', 0.04)
        self.declare_parameter('max_angular_z', 2.5)
        self.declare_parameter('integral_clamp', 1.0)
        self.declare_parameter('search_hold_time', 0.4)
        self.declare_parameter('search_sweep_speed', 0.6)
        self.declare_parameter('search_sweep_angular', 1.2)
        self.declare_parameter('error_topic', '/line_sensor')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.controller = PIDController(
            Kp=self.get_parameter('Kp').value,
            Ki=self.get_parameter('Ki').value,
            Kd=self.get_parameter('Kd').value,
            base_linear_speed=self.get_parameter('base_linear_speed').value,
            min_linear_speed=self.get_parameter('min_linear_speed').value,
            max_angular_z=self.get_parameter('max_angular_z').value,
            integral_clamp=self.get_parameter('integral_clamp').value,
            search_hold_time=self.get_parameter('search_hold_time').value,
            search_sweep_speed=self.get_parameter('search_sweep_speed').value,
            search_sweep_angular=self.get_parameter('search_sweep_angular').value,
        )
        self.controller.reset()  # clear any stale integral/derivative state

        error_topic = self.get_parameter('error_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.add_on_set_parameters_callback(self.parameters_callback)

        # Reliable QoS for the Gazebo DiffDrive plugin.
        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        # Best-Effort on the input to prevent control lag.
        self.subscription = self.create_subscription(
            Float32, error_topic, self.error_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f"pid_controller_node ready. Listening on '{error_topic}', "
            f"driving '{cmd_vel_topic}'. Tune live via rqt or CLI.")

    def parameters_callback(self, params):
        # Only accept params that map to a real controller attribute; reject
        # unknown ones loudly instead of silently reporting success.
        for param in params:
            if hasattr(self.controller, param.name):
                setattr(self.controller, param.name, param.value)
                self.get_logger().info(f"Updated {param.name} to {param.value}")
            else:
                self.get_logger().warn(f"Rejected {param.name}: not a controller parameter")
                return SetParametersResult(successful=False, reason=f"Unknown parameter {param.name}")
        return SetParametersResult(successful=True)
    def error_callback(self, msg: Float32):
        error = float(msg.data)
        # NaN = line lost (sentinel published by sensor_node).
        if math.isnan(error):
            error = None
        linear_x, angular_z = self.controller.update(error)

        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
