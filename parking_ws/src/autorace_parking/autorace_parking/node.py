"""T자 주차 미션 → /mission/parking/command_03.

입력: /yolo/parking_sign (Bool), /lane/target (T_PARK 주차선 거리), /mission/lane, /encoder/distance
출력: /mission/parking/command_03 (ARMED 부터 계속 발행, PARKED 이후 속도 0 유지), /parking/state
"""

import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, Float64, String

from .core import Parking

T_PARK = 6


class ParkingNode(Node):

    def __init__(self):
        super().__init__('parking_node')
        d = self.declare_parameter
        self.core = Parking(
            wheelbase=float(d('wheelbase', 0.26).value),
            slot_width=float(d('slot_width', 0.50).value),
            slot_index=float(d('slot_index', 1.5).value),
            turn_radius=float(d('turn_radius', 0.613).value),
            turn_offset=float(d('turn_offset', 0.0).value),
            turn_scale=float(d('turn_scale', 1.0).value),
            enter_distance=float(d('enter_distance', 0.35).value),
            approach_speed=float(d('approach_speed', 0.3).value),
            maneuver_speed=float(d('maneuver_speed', 0.25).value),
            max_steer=float(d('max_steer', 0.401).value),
            side=int(d('side', -1).value),
            arm_timeout_dist=float(d('arm_timeout_dist', 6.0).value))
        self.lane_timeout = float(d('lane_timeout', 0.3).value)
        self.lane = None
        self.lane_t = 0.0
        self.s = 0.0
        self.park_dist = -1.0
        self.pub = self.create_publisher(AckermannDriveStamped, '/mission/parking/command_03', 10)
        self.state_pub = self.create_publisher(String, '/parking/state', 10)
        self.create_subscription(AckermannDriveStamped, '/mission/lane', self.on_lane, 10)
        self.create_subscription(Bool, '/yolo/parking_sign', self.on_sign, 10)
        self.create_subscription(Float32MultiArray, '/lane/target', self.on_target, 10)
        self.create_subscription(Float64, '/encoder/distance', lambda m: setattr(self, 's', m.data), 10)
        self.create_timer(1.0 / 50.0, self.tick)
        self.last_state = None

    def on_lane(self, msg):
        self.lane = msg
        self.lane_t = time.monotonic()

    def on_sign(self, msg):
        if msg.data:
            self.core.on_sign(self.s)

    def on_target(self, msg):
        if len(msg.data) > T_PARK:
            self.park_dist = msg.data[T_PARK]

    def tick(self):
        now = time.monotonic()
        lane_ok = self.lane is not None and now - self.lane_t < self.lane_timeout
        out = self.core.step(self.s, self.park_dist,
                             self.lane.drive.steering_angle if lane_ok else 0.0,
                             self.lane.drive.speed if lane_ok else 0.0)
        if self.core.state != self.last_state:
            self.get_logger().info(f'parking: {self.last_state} → {self.core.state}')
            self.last_state = self.core.state
            self.state_pub.publish(String(data=self.core.state))
        if out is None:
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
        node = ParkingNode()
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
