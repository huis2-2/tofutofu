"""미션 명령 우선순위 mux → /drive/cmd.

입력: core.INPUTS 의 AckermannDriveStamped 토픽들
      /mux/enable (Bool) : false 면 정지 (start_enabled 기본 true)
출력: /drive/cmd (AckermannDriveStamped, 50 Hz), /mux/active_source (String, 바뀔 때)
"""

import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .core import INPUTS, Arbiter


class MuxNode(Node):

    def __init__(self):
        super().__init__('mux_node')
        d = self.declare_parameter
        self.arb = Arbiter(timeout=float(d('input_timeout', 0.25).value),
                           max_speed=float(d('max_speed', 2.0).value),
                           max_steer=float(d('max_steer', 0.6).value))
        self.enabled = bool(d('start_enabled', True).value)
        self.disabled = set(d('disabled_sources', ['']).value) - {''}
        rate = float(d('rate', 50.0).value)
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive/cmd', 10)
        self.src_pub = self.create_publisher(String, '/mux/active_source', 10)
        for source, topic in INPUTS:
            self.create_subscription(AckermannDriveStamped, topic,
                                     lambda m, s=source: self.on_cmd(s, m), 10)
        self.create_subscription(Bool, '/mux/enable', self.on_enable, 10)
        self.create_timer(1.0 / rate, self.tick)
        self.active = None
        if self.disabled:
            self.get_logger().info(f'비활성 미션: {sorted(self.disabled)}')

    def on_enable(self, msg):
        self.enabled = msg.data
        self.get_logger().info(f'mux enable = {msg.data}')

    def on_cmd(self, source, msg):
        self.arb.receive(source, msg.drive.speed, msg.drive.steering_angle, time.monotonic())

    def tick(self):
        source, speed, steer = self.arb.select(time.monotonic(), self.disabled)
        if not self.enabled:
            source, speed = 'disabled', 0.0
        if source != self.active:
            self.get_logger().info(f'active: {self.active} → {source}')
            self.active = source
            self.src_pub.publish(String(data=source))
        m = AckermannDriveStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.drive.speed = float(speed)
        m.drive.steering_angle = float(steer)
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MuxNode()
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
