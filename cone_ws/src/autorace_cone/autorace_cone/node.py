"""라바콘 미션 → /mission/cone/command_06 (자체 조향).

입력: /scan, /yolo/cone (Bool), /encoder/distance
출력: /mission/cone/command_06, /cone/active (Bool)
"""

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64

from .core import ConeCourse, clusters, scan_to_points


class ConeNode(Node):

    def __init__(self):
        super().__init__('cone_node')
        d = self.declare_parameter
        self.lidar = dict(range_min=float(d('range_min', 0.05).value),
                          range_max=float(d('range_max', 2.0).value),
                          lidar_x=float(d('lidar_x', 0.20).value),
                          lidar_yaw=float(d('lidar_yaw', 3.14159265).value),
                          flip=bool(d('lidar_flip', False).value))
        self.cluster_gap = float(d('cluster_gap', 0.08).value)
        self.max_size = float(d('max_cone_size', 0.18).value)
        self.core = ConeCourse(
            wheelbase=float(d('wheelbase', 0.26).value),
            roi_x=(float(d('roi_x_min', 0.05).value), float(d('roi_x_max', 1.6).value)),
            roi_y=float(d('roi_y', 1.0).value), lookahead=float(d('lookahead', 0.8).value),
            half_gap=float(d('half_gap', 0.30).value), min_cones=int(d('min_cones', 2).value),
            exit_distance=float(d('exit_distance', 0.6).value), speed=float(d('speed', 0.5).value),
            max_steer=float(d('max_steer', 0.45).value),
            steer_filter=float(d('steer_filter', 0.5).value),
            arm_distance=float(d('arm_distance', 3.0).value),
            require_yolo=bool(d('require_yolo', True).value))
        self.s = 0.0
        self.pub = self.create_publisher(AckermannDriveStamped, '/mission/cone/command_06', 10)
        self.active_pub = self.create_publisher(Bool, '/cone/active', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, '/yolo/cone', self.on_yolo, 10)
        self.create_subscription(Float64, '/encoder/distance', lambda m: setattr(self, 's', m.data), 10)
        self.was_active = False

    def on_yolo(self, msg):
        if msg.data:
            self.core.on_yolo(self.s)

    def on_scan(self, msg):
        pts = scan_to_points(np.array(msg.ranges), msg.angle_min, msg.angle_increment, **self.lidar)
        cones = clusters(pts, self.cluster_gap, self.max_size)
        out = self.core.step(self.s, cones)
        active = out is not None
        if active != self.was_active:
            self.get_logger().info(f'cone {"시작" if active else "종료"}')
            self.was_active = active
        self.active_pub.publish(Bool(data=active))
        if not active:
            return
        m = AckermannDriveStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.drive.speed = float(out[0])
        m.drive.steering_angle = float(out[1])
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ConeNode()
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
