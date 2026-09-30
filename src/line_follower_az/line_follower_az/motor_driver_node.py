#!/usr/bin/env python3
"""motor_driver_node.py — ROS2 wrapper around motor_model.DifferentialDrive.

Role in the architecture (same as the original):
    Publishes : /wheel_velocity (std_msgs/Float32MultiArray, [left, right] rad/s)
    Subscribes: /cmd_vel        (geometry_msgs/Twist)

In Gazebo the DiffDrive plugin consumes /cmd_vel directly; this node stays
active to provide wheel-command telemetry and keep architectural parity with a
future physical robot.
"""

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from line_follower_az.motor_model import DifferentialDrive


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('wheel_separation', 0.15)
        self.declare_parameter('max_wheel_speed', 12.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('wheel_velocity_topic', '/wheel_velocity')

        self.drive = DifferentialDrive(
            wheel_radius=self.get_parameter('wheel_radius').value,
            wheel_separation=self.get_parameter('wheel_separation').value,
            max_wheel_speed=self.get_parameter('max_wheel_speed').value,
        )

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        wheel_velocity_topic = self.get_parameter('wheel_velocity_topic').value

        self.add_on_set_parameters_callback(self.parameters_callback)

        # Reliable QoS so monitor_node can subscribe to this telemetry.
        self.publisher = self.create_publisher(
            Float32MultiArray, wheel_velocity_topic, 10)
        self.subscription = self.create_subscription(
            Twist, cmd_vel_topic, self.cmd_vel_callback, 10)

        self.get_logger().info(
            f"motor_driver_node ready. Converting '{cmd_vel_topic}' into "
            f"left/right wheel speeds on '{wheel_velocity_topic}'.")

    def parameters_callback(self, params):
        for param in params:
            if hasattr(self.drive, param.name):
                setattr(self.drive, param.name, param.value)
                self.get_logger().info(f"Updated {param.name} to {param.value}")
            else:
                self.get_logger().warn(f"Rejected {param.name}: not a drive parameter")
                return SetParametersResult(successful=False, reason=f"Unknown parameter {param.name}")
        return SetParametersResult(successful=True)

    def cmd_vel_callback(self, msg: Twist):
        v_left, v_right = self.drive.twist_to_wheels(msg.linear.x, msg.angular.z)

        out = Float32MultiArray()
        out.data = [v_left, v_right]
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
