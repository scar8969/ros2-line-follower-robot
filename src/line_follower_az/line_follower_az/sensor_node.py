#!/usr/bin/env python3
"""sensor_node.py — ROS2 wrapper around sensor_model.compute_line_error().

Role in the architecture (same as the original):
    Publishes : /line_sensor   (std_msgs/Float32)
    Subscribes: /camera/image_raw (sensor_msgs/Image), bridged in from Gazebo

The camera image is treated exactly like a 5-channel IR sensor array: the ROI
strip is split into 5 zones, each votes on whether it sees the dark line, and
the votes are combined into a normalized error in [-1, 1]. When the line is
lost, the last known error is held so the robot keeps steering the way it was.
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from line_follower_az.sensor_model import compute_line_error


class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')

        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('line_sensor_topic', '/line_sensor')
        self.declare_parameter('roi_top_fraction', 0.55)
        self.declare_parameter('binary_threshold', 90)
        self.declare_parameter('num_zones', 5)

        camera_topic = self.get_parameter('camera_topic').value
        line_sensor_topic = self.get_parameter('line_sensor_topic').value

        self.bridge = CvBridge()
        self.last_error = 0.0
        self.have_seen_line = False

        # Best-Effort sensor QoS: drop stale frames instead of queueing them,
        # which keeps control latency low around sharp corners.
        self.publisher = self.create_publisher(
            Float32, line_sensor_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            Image, camera_topic, self.image_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f"sensor_node ready. Reading '{camera_topic}', "
            f"publishing line position to '{line_sensor_topic}'.")

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f"Could not convert image: {exc}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        error, _ = compute_line_error(
            gray,
            num_zones=int(self.get_parameter('num_zones').value),
            roi_top_fraction=self.get_parameter('roi_top_fraction').value,
            binary_threshold=int(self.get_parameter('binary_threshold').value),
        )

        if error is None:
            # Line lost: publish NaN so the PID controller can enter its
            # search-recovery state (Float32 has no null).
            error = float('nan')
        else:
            self.last_error = error
            self.have_seen_line = True

        out = Float32()
        out.data = float(error)
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
