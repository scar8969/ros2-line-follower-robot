#!/usr/bin/env python3
"""logger_node.py — ROS2 CSV logger.

Role in the architecture (same as the original):
    Subscribes: /line_sensor, /cmd_vel
    Publishes : nothing

Every 1/log_rate_hz seconds writes one CSV row: timestamp, line_error,
linear_x, angular_z. Gives you the raw data behind the performance metrics
without parsing a rosbag2 file.
"""

import csv
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32


class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')

        self.declare_parameter('log_directory', '~/ros2_az_logs')
        self.declare_parameter('log_rate_hz', 10.0)

        log_dir = os.path.expanduser(self.get_parameter('log_directory').value)
        os.makedirs(log_dir, exist_ok=True)

        run_name = time.strftime('run_%Y%m%d_%H%M%S.csv')
        self.log_path = os.path.join(log_dir, run_name)
        self._file = open(self.log_path, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(['timestamp_s', 'line_error', 'linear_x', 'angular_z'])

        self._latest_error = 0.0
        self._latest_linear_x = 0.0
        self._latest_angular_z = 0.0
        self._start_time = time.time()

        self.create_subscription(Float32, '/line_sensor', self._error_cb, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        log_rate_hz = self.get_parameter('log_rate_hz').value
        period = 1.0 / log_rate_hz if log_rate_hz > 0 else 0.1
        self._timer = self.create_timer(period, self._write_row)

        self.get_logger().info(f"logger_node ready. Writing to: {self.log_path}")

    def _error_cb(self, msg: Float32):
        self._latest_error = float(msg.data)

    def _cmd_vel_cb(self, msg: Twist):
        self._latest_linear_x = float(msg.linear.x)
        self._latest_angular_z = float(msg.angular.z)

    def _write_row(self):
        t = time.time() - self._start_time
        self._writer.writerow([f"{t:.3f}", self._latest_error,
                                self._latest_linear_x, self._latest_angular_z])
        self._file.flush()

    def destroy_node(self):
        try:
            self._file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
