#!/usr/bin/env python3
"""Approximate D435 disparity noise and register native depth points to RGB.

This is not a stereo matcher or a measured model of a particular camera.
Invalid/occluded geometry stays invalid. Noise is sampled in disparity space,
so depth uncertainty grows with distance, followed by depth-unit quantization.
"""
from collections import deque
import copy

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import Buffer, TransformListener, TransformException
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation


def model_depth(depth, fx, baseline, sigma, units, minimum, maximum, rng):
    valid = np.isfinite(depth) & (depth >= minimum) & (depth <= maximum)
    z = np.where(valid, depth, 1.0).astype(np.float32)
    disparity = fx * baseline / z
    if sigma > 0:
        disparity += rng.normal(0, sigma, depth.shape).astype(np.float32)
    valid &= disparity > 0
    z = fx * baseline / np.maximum(disparity, 1e-6)
    z = np.rint(z / units) * units
    valid &= (z >= minimum) & (z <= maximum)
    return np.where(valid, z, np.nan).astype(np.float32)


def registered_points(depth, depth_k, color_k, color, rotation, translation):
    """XYZ in color optical coordinates, with RGB sampled from the real RGB render."""
    h, w = depth.shape
    yy, xx = np.indices((h, w), dtype=np.float32)
    xyz = np.stack(((xx - depth_k[0, 2]) * depth / depth_k[0, 0],
                    (yy - depth_k[1, 2]) * depth / depth_k[1, 1], depth), axis=-1)
    xyz = xyz @ rotation.T + translation
    valid = np.isfinite(xyz).all(axis=-1) & (xyz[..., 2] > 0)
    z = np.where(valid, xyz[..., 2], 1)
    u = np.rint(np.where(valid, xyz[..., 0] / z * color_k[0, 0] + color_k[0, 2], -1)).astype(np.int32)
    v = np.rint(np.where(valid, xyz[..., 1] / z * color_k[1, 1] + color_k[1, 2], -1)).astype(np.int32)
    inside = valid & (u >= 0) & (u < color.shape[1]) & (v >= 0) & (v < color.shape[0])
    packed = np.zeros((h, w), dtype=np.uint32)
    colors = color[v[inside], u[inside]].astype(np.uint32)
    packed[inside] = (colors[:, 0] << 16) | (colors[:, 1] << 8) | colors[:, 2]
    result = np.empty((h, w), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
    for i, field in enumerate(('x', 'y', 'z')):
        result[field] = xyz[..., i]
    result['rgb'] = packed
    return result


class D435DepthProcessing(Node):
    def __init__(self):
        super().__init__('d435_depth_processing')
        for key, value in {'depth_min_m': 0.28, 'depth_max_m': 10.0,
                           'stereo_baseline_m': 0.05, 'disparity_noise_stddev_px': 0.08,
                           'depth_units_m': 0.001, 'pointcloud_rate_hz': 10.0,
                           'color_frame': 'camera_color_optical_frame',
                           'depth_frame': 'camera_depth_optical_frame'}.items():
            self.declare_parameter(key, value)
        self.config = {key: self.get_parameter(key).value for key in
                       ('depth_min_m', 'depth_max_m', 'stereo_baseline_m', 'disparity_noise_stddev_px',
                        'depth_units_m', 'pointcloud_rate_hz', 'color_frame', 'depth_frame')}
        self.bridge = CvBridge()
        self.rng = np.random.default_rng()
        self.colors = deque(maxlen=5)
        self.depth_info = self.color_info = None
        self.last_cloud = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(Image, '/piper/camera/image_raw', lambda msg: self.colors.append(msg), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/piper/camera/camera_info', self.on_color_info, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/piper/camera/depth/camera_info', self.on_depth_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/piper/camera/depth/ideal_image', self.process, qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, '/piper/camera/depth/image_raw', qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, '/piper/camera/points_reframed', 2)
        self.legacy_pub = self.create_publisher(PointCloud2, '/piper/camera/points', 2)
        self.get_logger().info('D435 depth: 1 mm units; approximate 0.08 px disparity noise; RGB-registered cloud up to 10 Hz.')

    def on_color_info(self, msg):
        self.color_info = msg

    def on_depth_info(self, msg):
        self.depth_info = msg

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def process(self, msg):
        if self.depth_info is None:
            return
        raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        if raw.shape != (self.depth_info.height, self.depth_info.width):
            return
        k = np.array(self.depth_info.k, dtype=np.float32).reshape(3, 3)
        cfg = self.config
        depth = model_depth(raw, k[0, 0], cfg['stereo_baseline_m'], cfg['disparity_noise_stddev_px'],
                            cfg['depth_units_m'], cfg['depth_min_m'], cfg['depth_max_m'], self.rng)
        if self.depth_pub.get_subscription_count():
            units = np.nan_to_num(depth / cfg['depth_units_m'], nan=0).round().astype(np.uint16)
            out = self.bridge.cv2_to_imgmsg(units, encoding='16UC1')
            out.header = copy.deepcopy(msg.header)
            out.header.frame_id = cfg['depth_frame']
            self.depth_pub.publish(out)
        stamp = self.stamp(msg)
        if self.last_cloud is not None and 0 <= stamp - self.last_cloud < 1 / cfg['pointcloud_rate_hz']:
            return
        if not (self.cloud_pub.get_subscription_count() or self.legacy_pub.get_subscription_count()):
            return
        if not self.colors or self.color_info is None:
            return
        color_msg = min(self.colors, key=lambda m: abs(self.stamp(m) - stamp))
        if abs(self.stamp(color_msg) - stamp) > 0.05:
            return
        try:
            tf = self.tf.lookup_transform(cfg['color_frame'], cfg['depth_frame'], Time()).transform
        except TransformException:
            return
        q, t = tf.rotation, tf.translation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix().astype(np.float32)
        translation = np.array([t.x, t.y, t.z], dtype=np.float32)
        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='rgb8')
        ck = np.array(self.color_info.k, dtype=np.float32).reshape(3, 3)
        points = registered_points(depth, k, ck, color, rotation, translation)
        cloud = PointCloud2()
        cloud.header = copy.deepcopy(msg.header)
        cloud.header.frame_id = cfg['color_frame']
        cloud.height, cloud.width = depth.shape
        cloud.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(('x', 'y', 'z', 'rgb'))]
        cloud.point_step = 16
        cloud.row_step = cloud.width * 16
        cloud.is_dense = False
        cloud.data = points.tobytes()
        if self.cloud_pub.get_subscription_count():
            self.cloud_pub.publish(cloud)
        if self.legacy_pub.get_subscription_count():
            self.legacy_pub.publish(cloud)
        self.last_cloud = stamp


def main():
    rclpy.init()
    node = D435DepthProcessing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
