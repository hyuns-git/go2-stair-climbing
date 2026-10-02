#!/usr/bin/env python3
"""
scan_maker_l1 — Go2 내장 L1 라이다 기반 /scan 생성 노드
(수정: 스캔-TF 타이밍 동기화 버그 수정, 2026-08-14)

버그였던 것: scan_timer(10Hz)와 tf_timer(50Hz)가 독립적으로 self.latest_odom을
각자 다른 순간에 읽어 발행 -> 스캔 점의 실제 각도와 slam_toolbox가 TF로 조회하는
각도가 회전 중 어긋남 -> 회전 지점에 스캔이 번짐/뭉침.

고침: publish_scan에서 스캔을 만들 때 쓴 (ox,oy,yaw)와 동일한 stamp로
그 자리에서 즉시 TF도 함께 발행한다 (_broadcast_tf 공용 함수).
tf_timer(50Hz)는 시각화용 부드러운 갱신 목적으로만 남겨두되,
스캔 시점의 "정확히 일치하는" TF 샘플은 이제 publish_scan이 보장한다.
"""

import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def parse_pointcloud2_xyz(msg: PointCloud2) -> np.ndarray:
    fields = {f.name: f for f in msg.fields}
    for req in ('x', 'y', 'z'):
        if req not in fields:
            raise ValueError(f"PointCloud2에 '{req}' 필드가 없다: {list(fields.keys())}")

    n_points = msg.width * msg.height
    if n_points == 0:
        return np.empty((0, 3), dtype=np.float64)

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    raw = raw[: n_points * msg.point_step].reshape(n_points, msg.point_step)

    def read_f32(name):
        f = fields[name]
        if f.datatype != PointField.FLOAT32:
            raise ValueError(f"'{name}' datatype={f.datatype}, FLOAT32 아님")
        return raw[:, f.offset:f.offset + 4].copy().view('<f4').reshape(-1)

    x, y, z = read_f32('x'), read_f32('y'), read_f32('z')
    return np.stack([x, y, z], axis=1).astype(np.float64)


def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quat_inverse(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


class ScanMakerL1(Node):
    def __init__(self):
        super().__init__('scan_maker_l1')

        self.declare_parameter('accumulate_sec', 0.75)
        self.declare_parameter('max_frames', 10)
        self.declare_parameter('min_height', 0.05)
        self.declare_parameter('max_height', 0.60)
        self.declare_parameter('num_beams', 180)
        self.declare_parameter('range_min', 0.30)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('scan_rate', 10.0)
        self.declare_parameter('tf_rate', 50.0)
        self.declare_parameter('min_points_per_scan', 150)
        self.declare_parameter('cloud_topic', '/utlidar/cloud_deskewed')
        self.declare_parameter('odom_topic', '/utlidar/robot_odom')

        self.num_beams = self.get_parameter('num_beams').value
        self.angle_increment = 2.0 * math.pi / self.num_beams

        max_frames = self.get_parameter('max_frames').value
        self.frame_buffer = deque(maxlen=max_frames)

        self.latest_odom = None

        self.cloud_sub = self.create_subscription(
            PointCloud2, self.get_parameter('cloud_topic').value,
            self.cloud_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self.odom_callback, qos_profile_sensor_data)

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        scan_period = 1.0 / self.get_parameter('scan_rate').value
        tf_period = 1.0 / self.get_parameter('tf_rate').value
        self.scan_timer = self.create_timer(scan_period, self.publish_scan)
        self.tf_timer = self.create_timer(tf_period, self.publish_tf_and_odom)

        self.get_logger().info(
            f"scan_maker_l1: {self.num_beams} beams "
            f"({math.degrees(self.angle_increment):.2f} deg), "
            f"accum {self.get_parameter('accumulate_sec').value}s, "
            f"z=[{self.get_parameter('min_height').value},{self.get_parameter('max_height').value}] "
            f"[scan-TF sync fix applied]"
        )

    def cloud_callback(self, msg: PointCloud2):
        p = parse_pointcloud2_xyz(msg)
        if p.shape[0] == 0:
            return

        ok = np.isfinite(p).all(axis=1)
        ok &= ~((np.abs(p[:, 0]) < 1e-6) & (np.abs(p[:, 1]) < 1e-6) & (np.abs(p[:, 2]) < 1e-6))
        p = p[ok]
        if p.shape[0] == 0:
            return

        min_h = self.get_parameter('min_height').value
        max_h = self.get_parameter('max_height').value
        hmask = (p[:, 2] >= min_h) & (p[:, 2] <= max_h)
        p = p[hmask]

        stamp_sec = self.get_clock().now().nanoseconds * 1e-9
        self.frame_buffer.append((p, stamp_sec))

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.latest_odom = (pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w, msg.twist)

    def _broadcast_tf(self, stamp, ox, oy, oz, yaw, q_remaining):
        """스캔/odom 발행과 동일한 (ox,oy,yaw)로 TF를 쏜다 — 동기화 핵심."""
        q_yaw = yaw_to_quat(yaw)

        t1 = TransformStamped()
        t1.header.stamp = stamp
        t1.header.frame_id = 'odom'
        t1.child_frame_id = 'base_footprint'
        t1.transform.translation.x = ox
        t1.transform.translation.y = oy
        t1.transform.translation.z = 0.0
        (t1.transform.rotation.x, t1.transform.rotation.y,
         t1.transform.rotation.z, t1.transform.rotation.w) = q_yaw

        t2 = TransformStamped()
        t2.header.stamp = stamp
        t2.header.frame_id = 'base_footprint'
        t2.child_frame_id = 'base_link'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = oz
        (t2.transform.rotation.x, t2.transform.rotation.y,
         t2.transform.rotation.z, t2.transform.rotation.w) = q_remaining

        self.tf_broadcaster.sendTransform([t1, t2])

    def publish_scan(self):
        if self.latest_odom is None or len(self.frame_buffer) == 0:
            return

        accumulate_sec = self.get_parameter('accumulate_sec').value
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        pts_list = [pts for pts, t in self.frame_buffer if (now_sec - t) <= accumulate_sec]
        if not pts_list:
            return
        pts = np.concatenate(pts_list, axis=0)

        min_pts = self.get_parameter('min_points_per_scan').value
        if pts.shape[0] < min_pts:
            self.get_logger().warn(
                f"scan_maker_l1: 누적 포인트 {pts.shape[0]} < {min_pts}, 스킵",
                throttle_duration_sec=2.0)
            return

        # ★ 이 스캔에 쓸 pose를 여기서 딱 한 번만 스냅샷 — TF도 이걸 그대로 쓴다
        ox, oy, oz, qx, qy, qz, qw, _ = self.latest_odom
        yaw = quat_to_yaw(qx, qy, qz, qw)
        q_yaw = yaw_to_quat(yaw)
        q_full = (qx, qy, qz, qw)
        q_remaining = quat_multiply(quat_inverse(q_yaw), q_full)

        dx = pts[:, 0] - ox
        dy = pts[:, 1] - oy
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        bx = dx * cos_y + dy * sin_y
        by = -dx * sin_y + dy * cos_y

        r = np.hypot(bx, by)
        ang = np.arctan2(by, bx)

        range_min = self.get_parameter('range_min').value
        range_max = self.get_parameter('range_max').value
        rmask = (r >= range_min) & (r <= range_max)
        r, ang = r[rmask], ang[rmask]

        n = self.num_beams
        ranges = np.full(n, math.inf, dtype=np.float64)
        if r.shape[0] > 0:
            bin_idx = np.floor((ang + math.pi) / self.angle_increment).astype(np.int64)
            bin_idx = np.clip(bin_idx, 0, n - 1)
            np.minimum.at(ranges, bin_idx, r)

        # ★ 이 stamp를 스캔과 TF가 공유한다
        stamp = self.get_clock().now().to_msg()

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'base_footprint'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi - self.angle_increment
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.get_parameter('scan_rate').value
        scan.range_min = range_min
        scan.range_max = range_max
        scan.ranges = ranges.tolist()
        self.scan_pub.publish(scan)

        # ★ 핵심 수정: 스캔과 정확히 같은 stamp/pose로 TF도 즉시 발행
        self._broadcast_tf(stamp, ox, oy, oz, yaw, q_remaining)

    def publish_tf_and_odom(self):
        """50Hz 부드러운 갱신용 (시각화 등). 스캔 매칭 정확도에는
        publish_scan의 동기화 발행이 이미 보장하므로 이건 보조 역할."""
        if self.latest_odom is None:
            return
        ox, oy, oz, qx, qy, qz, qw, twist = self.latest_odom
        yaw = quat_to_yaw(qx, qy, qz, qw)
        q_yaw = yaw_to_quat(yaw)
        q_full = (qx, qy, qz, qw)
        q_remaining = quat_multiply(quat_inverse(q_yaw), q_full)

        stamp = self.get_clock().now().to_msg()
        self._broadcast_tf(stamp, ox, oy, oz, yaw, q_remaining)

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = ox
        odom_msg.pose.pose.position.y = oy
        odom_msg.pose.pose.position.z = 0.0
        (odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
         odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w) = q_yaw
        odom_msg.twist = twist
        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMakerL1()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
