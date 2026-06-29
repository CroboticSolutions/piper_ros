#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


def _field_offset(msg: PointCloud2, name: str) -> int:
    for field in msg.fields:
        if field.name == name:
            return int(field.offset)
    raise ValueError(f"PointCloud2 field {name!r} is missing")


class PointCloudReframe(Node):
    def __init__(self):
        super().__init__("pointcloud_reframe")
        self.declare_parameter("input_topic", "/piper/camera/points")
        self.declare_parameter("output_topic", "/piper/camera/points_reframed")
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("xyz_transform", "identity")

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.xyz_transform = self.get_parameter("xyz_transform").get_parameter_value().string_value
        self._warned_unknown_transform = False

        self.sub = self.create_subscription(PointCloud2, input_topic, self.cb, 10)
        self.pub = self.create_publisher(PointCloud2, output_topic, 10)

    def _apply_xyz_transform(self, msg: PointCloud2):
        if self.xyz_transform in ("", "identity"):
            return
        if self.xyz_transform != "gazebo_camera_to_optical":
            if not self._warned_unknown_transform:
                self._warned_unknown_transform = True
                self.get_logger().warn(f"Unknown xyz_transform={self.xyz_transform!r}; publishing identity")
            return
        if msg.height <= 0 or msg.width <= 0:
            return

        data = bytearray(msg.data)
        dtype = np.dtype((">" if msg.is_bigendian else "<") + "f4")
        shape = (int(msg.height), int(msg.width))
        strides = (int(msg.row_step), int(msg.point_step))
        xs = np.ndarray(shape=shape, dtype=dtype, buffer=data, offset=_field_offset(msg, "x"), strides=strides)
        ys = np.ndarray(shape=shape, dtype=dtype, buffer=data, offset=_field_offset(msg, "y"), strides=strides)
        zs = np.ndarray(shape=shape, dtype=dtype, buffer=data, offset=_field_offset(msg, "z"), strides=strides)

        raw_x = xs.copy()
        raw_y = ys.copy()
        raw_z = zs.copy()
        xs[:, :] = -raw_y
        ys[:, :] = -raw_z
        zs[:, :] = raw_x
        msg.data = bytes(data)

    def cb(self, msg: PointCloud2):
        self._apply_xyz_transform(msg)
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)



def main():
    rclpy.init()
    node = PointCloudReframe()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
