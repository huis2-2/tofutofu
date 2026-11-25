"""보행자 표지/횡단보도 미션 → /mission/crosswalk/command_01.

입력: /mission/lane, /lane/target (T_CROSS = 앞바퀴→횡단보도 거리), /encoder/distance, /encoder/speed,
      /yolo/signal (신호등이 보이면 그 횡단보도는 signal 미션 담당이라 무시)
출력: /mission/crosswalk/command_01  (활성일 때만 발행. lane 조향 그대로, 속도만 바꿈)
      /crosswalk/state (std_msgs/String)
"""

import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Float64, Int8, String

from .core import CrosswalkMission, StopAtLine

T_CROSS = 5
T_STOP = 4


class CrosswalkNode(Node):

    def __init__(self):
        super().__init__('crosswalk_node')
        d = self.declare_parameter
        stop = StopAtLine(stop_before=float(d('stop_before', 0.05).value),
                          decel=float(d('decel', 1.5).value),
                          creep_speed=float(d('creep_speed', 0.12).value),
                          creep_dist=float(d('creep_dist', 0.10).value),
                          tolerance=float(d('tolerance', 0.01).value),
                          max_measure=float(d('max_measure', 1.2).value))
        self.use_stop_line = bool(d('use_stop_line', False).value)
        self.mission = CrosswalkMission(
            stop, trigger_dist=float(d('trigger_dist', 1.0).value),
            hold_sec=float(d('hold_sec', 3.5).value),
            pass_distance=float(d('pass_distance', 0.8).value),
            max_count=int(d('max_count', 1).value),
            signal_ignore_sec=float(d('signal_ignore_sec', 4.0).value))
        self.lane_timeout = float(d('lane_timeout', 0.3).value)
        self.lane = None
        self.lane_t = 0.0
        self.dist = -1.0
        self.s = 0.0
        self.v = 0.0
        self.pub = self.create_publisher(AckermannDriveStamped, '/mission/crosswalk/command_01', 10)
        self.state_pub = self.create_publisher(String, '/crosswalk/state', 10)
        self.create_subscription(AckermannDriveStamped, '/mission/lane', self.on_lane, 10)
        self.create_subscription(Float32MultiArray, '/lane/target', self.on_target, 10)
        self.create_subscription(Float64, '/encoder/distance', lambda m: setattr(self, 's', m.data), 10)
        self.create_subscription(Float32, '/encoder/speed', lambda m: setattr(self, 'v', m.data), 10)
        self.create_subscription(Int8, '/yolo/signal', lambda m: self.mission.on_signal(time.monotonic()), 10)
        self.create_timer(1.0 / 30.0, self.tick)
        self.last_state = None

    def on_lane(self, msg):
        self.lane = msg
        self.lane_t = time.monotonic()

    def on_target(self, msg):
        if len(msg.data) > T_CROSS:
            cw = msg.data[T_CROSS]
            # 정지선(horizontal_line)이 횡단보도 바로 앞에 있으면 그것을 기준으로 할 수도 있다
            if self.use_stop_line and 0 <= msg.data[T_STOP] < (cw if cw >= 0 else 9):
                cw = msg.data[T_STOP]
            self.dist = cw

    def tick(self):
        now = time.monotonic()
        lane_ok = self.lane is not None and now - self.lane_t < self.lane_timeout
        v_lane = self.lane.drive.speed if lane_ok else 0.0
        v = self.mission.step(now, self.s, self.v, self.dist, v_lane)
        if self.mission.state != self.last_state:
            self.get_logger().info(f'crosswalk: {self.last_state} → {self.mission.state}')
            self.last_state = self.mission.state
            self.state_pub.publish(String(data=self.mission.state))
        if v is None:
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
        node = CrosswalkNode()
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
