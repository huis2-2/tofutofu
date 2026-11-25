"""차선 추종 제어: Pure Pursuit (lookahead ∝ 속도) + 곡률 기반 속도 → /mission/lane.

입력
  /lane/path            nav_msgs/Path
  /lane/target          std_msgs/Float32MultiArray (valid, heading 사용)
  /encoder/speed        std_msgs/Float32 (m/s) → lookahead
  /encoder/distance     std_msgs/Float64
  /yolo/no_speed_limit  std_msgs/Bool → boost_distance 동안 v_boost
출력
  /mission/lane         ackermann_msgs/AckermannDriveStamped (speed m/s, steering_angle rad, + 왼쪽)
"""

import time

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, Float64

from .control_core import ControlParams, LaneController
from .lane_core import T_HEADING, T_VALID


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')
        cp = ControlParams()
        for name in cp.__dataclass_fields__:
            setattr(cp, name, float(self.declare_parameter(name, getattr(cp, name)).value))
        self.ctrl = LaneController(cp)
        self.boost_distance = float(self.declare_parameter('boost_distance', 3.0).value)
        self.input_timeout = float(self.declare_parameter('input_timeout', 0.3).value)

        self.path = None
        self.speed = 0.0
        self.distance = 0.0
        self.boost_until = -1.0
        self.last_t = None
        self.last_input = 0.0

        self.pub = self.create_publisher(AckermannDriveStamped, '/mission/lane', 10)
        self.create_subscription(Path, '/lane/path', self.on_path, 10)
        self.create_subscription(Float32MultiArray, '/lane/target', self.on_target, 10)
        self.create_subscription(Float32, '/encoder/speed', self.on_speed, 10)
        self.create_subscription(Float64, '/encoder/distance', self.on_distance, 10)
        self.create_subscription(Bool, '/yolo/no_speed_limit', self.on_boost, 10)
        self.create_timer(0.05, self.watchdog)

    def on_speed(self, msg):
        self.speed = msg.data

    def on_distance(self, msg):
        self.distance = msg.data

    def on_boost(self, msg):
        if msg.data:
            self.boost_until = self.distance + self.boost_distance

    def on_path(self, msg):
        self.path = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])

    def on_target(self, msg):
        now = time.monotonic()
        self.last_input = now
        valid = len(msg.data) > T_VALID and msg.data[T_VALID] > 0.5
        heading = msg.data[T_HEADING] if valid else 0.0
        self.publish(now, valid, heading)

    def watchdog(self):
        now = time.monotonic()
        if now - self.last_input > self.input_timeout:
            self.publish(now, False, 0.0)  # 마스크가 끊기면 정지 쪽으로

    def publish(self, now, valid, heading):
        dt = 0.033 if self.last_t is None else min(0.2, max(1e-3, now - self.last_t))
        self.last_t = now
        boost = self.distance < self.boost_until
        v, steer, _ = self.ctrl.step(now, dt, valid, self.path if valid else None,
                                     heading, self.speed, boost)
        m = AckermannDriveStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.drive.speed = float(v)
        m.drive.steering_angle = float(steer)
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ControllerNode()
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
