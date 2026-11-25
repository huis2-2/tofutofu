"""정적 장애물 회피 → /mission/objavoid/command_05 + /lane/change.

입력: /scan (LaserScan), /yolo/fixed_obstacle (Bool), /lane/target (좌/우 점선 여부),
      /mission/lane, /encoder/distance
출력: /mission/objavoid/command_05 (활성일 때. lane 조향 + 감속), /lane/change (Int8), /objavoid/active
"""

import time

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, Float64, Int8

from .core import ObjAvoid, scan_to_points

T_LEFT_DASHED, T_RIGHT_DASHED = 10, 11


class ObjAvoidNode(Node):

    def __init__(self):
        super().__init__('objavoid_node')
        d = self.declare_parameter
        self.lidar = dict(range_min=float(d('range_min', 0.05).value),
                          range_max=float(d('range_max', 3.0).value),
                          lidar_x=float(d('lidar_x', 0.20).value),
                          lidar_yaw=float(d('lidar_yaw', 3.14159265).value),
                          flip=bool(d('lidar_flip', False).value))
        self.core = ObjAvoid(
            lane_width=float(d('lane_width', 0.55).value), front=float(d('front', 0.26).value),
            check_dist=float(d('check_dist', 0.9).value),
            corridor_half=float(d('corridor_half', 0.17).value),
            min_points=int(d('min_points', 4).value),
            change_cooldown=float(d('change_cooldown', 0.6).value),
            exit_distance=float(d('exit_distance', 0.8).value),
            avoid_speed=float(d('avoid_speed', 0.5).value),
            require_dashed=bool(d('require_dashed', True).value),
            prefer=int(d('prefer', 1).value),
            active_timeout_dist=float(d('active_timeout_dist', 2.5).value),
            require_yolo=bool(d('require_yolo', True).value))
        self.dashed_threshold = float(d('dashed_threshold', 0.3).value)
        self.lane_timeout = float(d('lane_timeout', 0.3).value)
        self.lane = None
        self.lane_t = 0.0
        self.s = 0.0
        self.left_dashed = False
        self.right_dashed = False
        self.pub = self.create_publisher(AckermannDriveStamped, '/mission/objavoid/command_05', 10)
        self.change_pub = self.create_publisher(Int8, '/lane/change', 10)
        self.active_pub = self.create_publisher(Bool, '/objavoid/active', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(AckermannDriveStamped, '/mission/lane', self.on_lane, 10)
        self.create_subscription(Bool, '/yolo/fixed_obstacle', self.on_yolo, 10)
        self.create_subscription(Float32MultiArray, '/lane/target', self.on_target, 10)
        self.create_subscription(Float64, '/encoder/distance', lambda m: setattr(self, 's', m.data), 10)
        self.was_active = False

    def on_lane(self, msg):
        self.lane = msg
        self.lane_t = time.monotonic()

    def on_yolo(self, msg):
        if msg.data:
            self.core.on_yolo(self.s)

    def on_target(self, msg):
        if len(msg.data) > T_RIGHT_DASHED:
            self.left_dashed = msg.data[T_LEFT_DASHED] > self.dashed_threshold
            self.right_dashed = msg.data[T_RIGHT_DASHED] > self.dashed_threshold

    def on_scan(self, msg):
        pts = scan_to_points(np.array(msg.ranges), msg.angle_min, msg.angle_increment, **self.lidar)
        now = time.monotonic()
        lane_ok = self.lane is not None and now - self.lane_t < self.lane_timeout
        v_lane = self.lane.drive.speed if lane_ok else 0.0
        v, change = self.core.step(self.s, pts, self.left_dashed, self.right_dashed, v_lane)
        if change:
            self.get_logger().info(f'장애물 → 차선 변경 {"왼쪽" if change > 0 else "오른쪽"}')
            self.change_pub.publish(Int8(data=change))
        active = v is not None
        if active != self.was_active:
            self.get_logger().info(f'objavoid {"시작" if active else "종료"}')
            self.was_active = active
        self.active_pub.publish(Bool(data=active))
        if not active:
            return
        m = AckermannDriveStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.drive.speed = float(v)
        m.drive.steering_angle = float(self.lane.drive.steering_angle) if lane_ok else 0.0
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObjAvoidNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
