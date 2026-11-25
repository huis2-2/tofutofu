"""서행 미션 (방지턱 / 서행 표지 / 마름모 / 지그재그 서행 유도선) → /mission/slow/command_04.

입력: /yolo/slow_sign (Bool), /lane/target (T_DIAMOND), /lane/zigzag (UInt8),
      /mission/lane, /encoder/distance
출력: /mission/slow/command_04 (활성일 때만. lane 조향 그대로, 속도만), /slow/active (Bool)

지그재그 구간의 '직진 헤딩 유지'는 lane_node 가 지그재그 경계를 직선으로 피팅해서 처리한다.
"""

import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, Float64, UInt8

from .core import SlowMission

T_DIAMOND = 7


class SlowNode(Node):

    def __init__(self):
        super().__init__('slow_node')
        d = self.declare_parameter
        self.mission = SlowMission(
            ratio=float(d('ratio', 0.45).value), max_speed=float(d('max_speed', 0.45).value),
            min_speed=float(d('min_speed', 0.15).value),
            hold_distance=float(d('hold_distance', 1.2).value),
            zigzag_hold_distance=float(d('zigzag_hold_distance', 0.4).value),
            diamond_trigger_dist=float(d('diamond_trigger_dist', 0.8).value))
        self.use_diamond = bool(d('use_diamond', True).value)
        self.use_zigzag = bool(d('use_zigzag', True).value)
        self.lane_timeout = float(d('lane_timeout', 0.3).value)
        self.lane = None
        self.lane_t = 0.0
        self.s = 0.0
        self.sign = False
        self.diamond = -1.0
        self.zigzag = 0
        self.active = False
        self.pub = self.create_publisher(AckermannDriveStamped, '/mission/slow/command_04', 10)
        self.active_pub = self.create_publisher(Bool, '/slow/active', 10)
        self.create_subscription(AckermannDriveStamped, '/mission/lane', self.on_lane, 10)
        self.create_subscription(Bool, '/yolo/slow_sign', self.on_sign, 10)
        self.create_subscription(Float32MultiArray, '/lane/target', self.on_target, 10)
        self.create_subscription(UInt8, '/lane/zigzag', lambda m: setattr(self, 'zigzag', m.data), 10)
        self.create_subscription(Float64, '/encoder/distance', lambda m: setattr(self, 's', m.data), 10)
        self.create_timer(1.0 / 30.0, self.tick)

    def on_lane(self, msg):
        self.lane = msg
        self.lane_t = time.monotonic()

    def on_sign(self, msg):
        self.sign = self.sign or msg.data  # tick 에서 소비

    def on_target(self, msg):
        if len(msg.data) > T_DIAMOND:
            self.diamond = msg.data[T_DIAMOND]

    def tick(self):
        now = time.monotonic()
        lane_ok = self.lane is not None and now - self.lane_t < self.lane_timeout
        v_lane = self.lane.drive.speed if lane_ok else 0.0
        v = self.mission.step(self.s, v_lane, self.sign,
                              self.diamond if self.use_diamond else -1.0,
                              self.zigzag if self.use_zigzag else 0)
        self.sign = False
        active = v is not None
        if active != self.active:
            self.get_logger().info(f'서행 {"시작 (" + self.mission.reason + ")" if active else "종료"}')
            self.active = active
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
        node = SlowNode()
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
