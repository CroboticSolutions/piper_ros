#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class PointCloudReframe(Node):
    def __init__(self):
        super().__init__("pointcloud_reframe")
        self.declare_parameter("input_topic", "/piper/camera/points")
        self.declare_parameter("output_topic", "/piper/camera/points_reframed")
        self.declare_parameter("frame_id", "piper/camera_link/camera")

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        self.sub = self.create_subscription(PointCloud2, input_topic, self.cb, 10)
        self.pub = self.create_publisher(PointCloud2, output_topic, 10)

    def cb(self, msg: PointCloud2):
        out = PointCloud2()
        out.header = msg.header
        out.header.frame_id = self.frame_id
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.data = msg.data
        out.is_dense = msg.is_dense
        self.pub.publish(out)


def main():
    rclpy.init()
    node = PointCloudReframe()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
