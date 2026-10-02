#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import TransformStamped
import tf2_ros

# K-12 확정 캘리브레이션 (base_footprint -> os_sensor)
CAL_X, CAL_Y, CAL_Z = 0.25, 0.0, 0.467
CAL_ROLL, CAL_PITCH, CAL_YAW = 0.0195, -0.0817, 0.0


def euler_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quat_conj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


class ScanMakerOS1(Node):
    def __init__(self):
        super().__init__('scan_maker_os1')

        self.declare_parameter('height_min', 0.05)
        self.declare_parameter('height_max', 1.00)
        self.declare_parameter('num_beams', 180)
        self.declare_parameter('max_range', 4.0)

        # base_footprint -> os_sensor 정적 변환 (4x4)
        R = euler_to_R(CAL_ROLL, CAL_PITCH, CAL_YAW)
        self.T_bf_sensor = np.eye(4)
        self.T_bf_sensor[:3, :3] = R
        self.T_bf_sensor[:3, 3] = [CAL_X, CAL_Y, CAL_Z]

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        self.create_subscription(Odometry, '/utlidar/robot_odom', self.odom_cb, 10)

        cloud_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2, '/ouster/points', self.cloud_cb, cloud_qos)

        self.get_logger().info('scan_maker_os1 started')

    def odom_cb(self, msg: Odometry):
        stamp = self.get_clock().now().to_msg()
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        q_yaw = yaw_to_quat(yaw)
        q_full = (q.x, q.y, q.z, q.w)
        q_remainder = quat_mul(quat_conj(q_yaw), q_full)  # base_footprint -> base_link 회전

        # odom -> base_footprint (yaw만, z=0)
        t1 = TransformStamped()
        t1.header.stamp = stamp
        t1.header.frame_id = 'odom'
        t1.child_frame_id = 'base_footprint'
        t1.transform.translation.x = msg.pose.pose.position.x
        t1.transform.translation.y = msg.pose.pose.position.y
        t1.transform.translation.z = 0.0
        t1.transform.rotation.x = q_yaw[0]
        t1.transform.rotation.y = q_yaw[1]
        t1.transform.rotation.z = q_yaw[2]
        t1.transform.rotation.w = q_yaw[3]

        # base_footprint -> base_link (z, roll, pitch)
        t2 = TransformStamped()
        t2.header.stamp = stamp
        t2.header.frame_id = 'base_footprint'
        t2.child_frame_id = 'base_link'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = msg.pose.pose.position.z
        t2.transform.rotation.x = q_remainder[0]
        t2.transform.rotation.y = q_remainder[1]
        t2.transform.rotation.z = q_remainder[2]
        t2.transform.rotation.w = q_remainder[3]

        self.tf_broadcaster.sendTransform([t1, t2])

        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_footprint'
        out.pose.pose.position.x = msg.pose.pose.position.x
        out.pose.pose.position.y = msg.pose.pose.position.y
        out.pose.pose.position.z = 0.0
        out.pose.pose.orientation.x = q_yaw[0]
        out.pose.pose.orientation.y = q_yaw[1]
        out.pose.pose.orientation.z = q_yaw[2]
        out.pose.pose.orientation.w = q_yaw[3]
        out.twist = msg.twist
        self.odom_pub.publish(out)

    def cloud_cb(self, msg: PointCloud2):
        cb_stamp = self.get_clock().now().to_msg()
        h_min = self.get_parameter('height_min').value
        h_max = self.get_parameter('height_max').value
        num_beams = self.get_parameter('num_beams').value
        max_range = self.get_parameter('max_range').value

        field_map = {f.name: f for f in msg.fields}
        dtype_lut = {2: np.uint8, 4: np.uint16, 5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}
        names = ['x', 'y', 'z']
        formats = [dtype_lut[field_map[n].datatype] for n in names]
        offsets = [field_map[n].offset for n in names]
        struct_dtype = np.dtype({'names': names, 'formats': formats, 'offsets': offsets, 'itemsize': msg.point_step})
        arr = np.frombuffer(msg.data, dtype=struct_dtype)
        pts = np.column_stack([arr['x'], arr['y'], arr['z']]).astype(np.float64)
        valid = np.isfinite(pts).all(axis=1)
        pts = pts[valid]
        if pts.shape[0] == 0:
            return

        # range 0 / 이상치 제거
        r3 = np.linalg.norm(pts, axis=1)
        pts = pts[(r3 > 0.01) & np.isfinite(r3)]
        if len(pts) == 0:
            return

        # os_sensor -> base_footprint 변환 (정적 캘리브레이션, 중력정렬 완료 상태)
        pts_h = np.hstack([pts, np.ones((len(pts), 1))])
        pts_bf = (self.T_bf_sensor @ pts_h.T).T[:, :3]

        # 높이 밴드 필터
        band = pts_bf[(pts_bf[:, 2] >= h_min) & (pts_bf[:, 2] <= h_max)]
        if len(band) == 0:
            self.get_logger().warn('높이 밴드 내 포인트 없음 — height_min/max 확인 필요', throttle_duration_sec=5.0)
            return

        angles = np.arctan2(band[:, 1], band[:, 0])
        ranges2d = np.linalg.norm(band[:, :2], axis=1)
        valid = (ranges2d > 0.01) & (ranges2d <= max_range) & np.isfinite(ranges2d)
        angles, ranges2d = angles[valid], ranges2d[valid]

        angle_min, angle_max = -math.pi, math.pi
        angle_increment = (angle_max - angle_min) / num_beams
        scan_ranges = np.full(num_beams, float('inf'))

        bins = ((angles - angle_min) / angle_increment).astype(int)
        bins = np.clip(bins, 0, num_beams - 1)
        np.minimum.at(scan_ranges, bins, ranges2d)

        coverage = float(np.isfinite(scan_ranges).sum()) / num_beams * 100.0

        scan = LaserScan()
        scan.header.stamp = cb_stamp
        scan.header.frame_id = 'base_footprint'
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.05
        scan.range_max = max_range
        scan.ranges = scan_ranges.tolist()
        self.scan_pub.publish(scan)

        self.get_logger().info(
            f'coverage={coverage:.1f}% beams={num_beams} band=[{h_min:.2f},{h_max:.2f}]',
            throttle_duration_sec=2.0)


def main():
    rclpy.init()
    rclpy.spin(ScanMakerOS1())


if __name__ == '__main__':
    main()
