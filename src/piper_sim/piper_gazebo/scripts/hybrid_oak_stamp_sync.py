#!/usr/bin/env python3
"""Republish OAK-D Pro W topics with sim-time stamps (hybrid Gazebo + real camera).

Driver keeps wall time on /oak_hw/oak/...
This node publishes /oak/... stamped from /clock.

Keep node name ``oak`` (namespaced under oak_hw) so DepthAI frame ids stay
oak_rgb_camera_optical_frame and match the Piper URDF hand-eye tree.

Pointclouds are opt-in (``enable_pointcloud``), same as FANUC hybrid D435.
"""

from __future__ import annotations

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


def _qos(depth: int = 5) -> QoSProfile:
    # DepthAI images/points are RELIABLE. WebRTC (server_ros.py IMAGE_QOS) also
    # requests RELIABLE — BEST_EFFORT out is silently dropped (no camera stream).
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
    )


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes")


class HybridOakStampSync(Node):
    def __init__(self) -> None:
        super().__init__("hybrid_oak_stamp_sync")
        self.declare_parameter("input_namespace", "/oak_hw/oak")
        self.declare_parameter("output_namespace", "/oak")
        self.declare_parameter("enable_pointcloud", False)
        inn = (
            self.get_parameter("input_namespace")
            .get_parameter_value()
            .string_value.rstrip("/")
        )
        out = (
            self.get_parameter("output_namespace")
            .get_parameter_value()
            .string_value.rstrip("/")
        )
        enable_pointcloud = _as_bool(self.get_parameter("enable_pointcloud").value)

        if not self.get_parameter("use_sim_time").get_parameter_value().bool_value:
            self.get_logger().error(
                "use_sim_time is false — hybrid stamp sync will NOT fix TF lookups."
            )

        qos_img = _qos(5)

        self._pub_color = self.create_publisher(Image, f"{out}/rgb/image_raw", qos_img)
        self._pub_color_info = self.create_publisher(
            CameraInfo, f"{out}/rgb/camera_info", qos_img
        )
        self._pub_depth = self.create_publisher(Image, f"{out}/stereo/image_raw", qos_img)

        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

        self._img_group = ReentrantCallbackGroup()

        self.create_subscription(
            Image, f"{inn}/rgb/image_raw", self._on_color, qos_img,
            callback_group=self._img_group,
        )
        self.create_subscription(
            CameraInfo, f"{inn}/rgb/camera_info", self._on_color_info, qos_img,
            callback_group=self._img_group,
        )
        self.create_subscription(
            Image, f"{inn}/stereo/image_raw", self._on_depth, qos_img,
            callback_group=self._img_group,
        )

        self._logged_points = False
        if enable_pointcloud:
            qos_points = _qos(2)
            self._pcl_group = MutuallyExclusiveCallbackGroup()
            self._pub_points = self.create_publisher(
                PointCloud2, f"{out}/rgbd/points", qos_points
            )
            self.create_subscription(
                PointCloud2, f"{inn}/rgbd/points", self._on_points, qos_points,
                callback_group=self._pcl_group,
            )

        self.get_logger().info(
            f"Hybrid OAK stamp sync: {inn}/… → {out}/… "
            f"(use_sim_time={self.get_parameter('use_sim_time').value}, "
            f"enable_pointcloud={enable_pointcloud})"
        )

    def _restamp(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        return msg

    def _on_color(self, msg: Image) -> None:
        self._pub_color.publish(self._restamp(msg))

    def _on_color_info(self, msg: CameraInfo) -> None:
        self._pub_color_info.publish(self._restamp(msg))

    def _on_depth(self, msg: Image) -> None:
        self._pub_depth.publish(self._restamp(msg))

    def _on_points(self, msg: PointCloud2) -> None:
        out = self._restamp(msg)
        if not self._logged_points:
            self._logged_points = True
            sim = out.header.stamp.sec + out.header.stamp.nanosec * 1e-9
            self.get_logger().info(
                f"First pointcloud restamp: sim_stamp={sim:.3f}s "
                f"frame={out.header.frame_id!r} "
                f"size={out.width}x{out.height}"
            )
        self._pub_points.publish(out)


def main() -> None:
    rclpy.init()
    node = HybridOakStampSync()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
