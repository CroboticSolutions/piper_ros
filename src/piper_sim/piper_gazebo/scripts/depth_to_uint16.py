#!/usr/bin/env python3

import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    import numpy as np
except ImportError:  # pragma: no cover - runtime dependency
    np = None


class DepthToUInt16(Node):
    def __init__(self):
        super().__init__("depth_to_uint16")
        self.declare_parameter("input_topic", "/piper/camera/depth/image_raw")
        self.declare_parameter("output_topic", "/piper/camera/depth/image_raw_uint16")

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        self.sub = self.create_subscription(Image, input_topic, self.cb, 10)
        self.pub = self.create_publisher(Image, output_topic, 10)

        if np is None:
            self.get_logger().error("numpy is required for depth conversion")

    def cb(self, msg: Image):
        if np is None:
            return
        if msg.encoding != "32FC1":
            self.get_logger().warn(f"Expected 32FC1 depth image, got {msg.encoding}")
            return

        depth = np.frombuffer(msg.data, dtype=np.float32)
        if msg.is_bigendian and sys.byteorder == "little":
            depth = depth.byteswap()

        # Convert meters to millimeters, clamp to uint16.
        depth_mm = np.clip(depth * 1000.0, 0.0, 65535.0).astype(np.uint16)
        # Replace NaNs with 0
        depth_mm[np.isnan(depth)] = 0

        out = Image()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.encoding = "16UC1"
        out.is_bigendian = 0
        out.step = msg.width * 2
        out.data = depth_mm.tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = DepthToUInt16()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
