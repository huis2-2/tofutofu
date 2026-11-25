"""RC카 구동 노드: /drive/cmd (속도 m/s, 조향 rad) → PCA9685 서보/ESC PWM.

입력: /drive/cmd (AckermannDriveStamped), /encoder/speed (Float32), /drive/estop (Bool, true = 정지)
출력: /drive/steering_us, /drive/throttle_us (Int32), /drive/steering (Float32, -1 오른쪽 ~ +1 왼쪽)
      → drive_record.py 와 같은 토픽이라 rosbag 으로 모방학습 데이터가 그대로 쌓인다.
안전: 명령이 cmd_timeout 동안 안 오면 중립. 종료 시 중립 + 조향 중앙.
"""

import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32

from .core import SpeedParams, SpeedPID, SteerParams, steer_ratio, steer_to_us
from .hardware import DryRunOutput, PCA9685Output


class DriveNode(Node):

    def __init__(self):
        super().__init__('drive_node')
        d = self.declare_parameter
        sp = SteerParams()
        for name in sp.__dataclass_fields__:
            setattr(sp, name, float(d(name, getattr(sp, name)).value))
        vp = SpeedParams()
        for name in vp.__dataclass_fields__:
            default = getattr(vp, name)
            setattr(vp, name, type(default)(d(name, default).value))
        self.sp, self.vp = sp, vp
        self.pid = SpeedPID(vp)
        self.cmd_timeout = float(d('cmd_timeout', 0.3).value)
        self.encoder_timeout = float(d('encoder_timeout', 0.3).value)
        rate = float(d('rate', 50.0).value)
        if bool(d('dry_run', False).value):
            self.out = DryRunOutput()
            self.get_logger().warning('dry_run: PWM 출력 안 함')
        else:
            self.out = PCA9685Output(
                bus=int(d('i2c_bus', 7).value), address=int(d('pca_address', 0x40).value),
                freq=float(d('pwm_freq', 300.0).value),
                steering_ch=int(d('steering_channel', 8).value),
                throttle_ch=int(d('throttle_channel', 9).value),
                mux_pin=int(d('mux_pin', 15).value))
        self.out.write(sp.center_us + sp.trim_us, vp.neutral_us)

        self.cmd = None
        self.cmd_t = 0.0
        self.speed = 0.0
        self.speed_t = 0.0
        self.estop = False
        self.last_t = time.monotonic()
        self.create_subscription(AckermannDriveStamped, '/drive/cmd', self.on_cmd, 10)
        self.create_subscription(Float32, '/encoder/speed', self.on_speed, 10)
        self.create_subscription(Bool, '/drive/estop', self.on_estop, 10)
        self.steer_us_pub = self.create_publisher(Int32, '/drive/steering_us', 10)
        self.throttle_us_pub = self.create_publisher(Int32, '/drive/throttle_us', 10)
        self.steer_pub = self.create_publisher(Float32, '/drive/steering', 10)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info('drive_node 시작')

    def on_cmd(self, msg):
        self.cmd = msg
        self.cmd_t = time.monotonic()

    def on_speed(self, msg):
        self.speed = msg.data
        self.speed_t = time.monotonic()

    def on_estop(self, msg):
        if msg.data != self.estop:
            self.get_logger().warning(f'E-STOP {"ON" if msg.data else "OFF"}')
        self.estop = msg.data

    def tick(self):
        now = time.monotonic()
        dt = max(1e-3, now - self.last_t)
        self.last_t = now
        alive = self.cmd is not None and now - self.cmd_t < self.cmd_timeout and not self.estop
        target = self.cmd.drive.speed if alive else 0.0
        steer = self.cmd.drive.steering_angle if alive else 0.0
        enc_ok = now - self.speed_t < self.encoder_timeout
        if alive and target > 0 and not enc_ok:
            self.get_logger().warning('엔코더 속도 없음 → 피드포워드만', throttle_duration_sec=2.0)
        throttle_us = self.pid.step(max(0.0, target), self.speed, dt, enc_ok)
        steer_us = steer_to_us(steer, self.sp)
        self.out.write(steer_us, throttle_us)
        self.steer_us_pub.publish(Int32(data=int(round(steer_us))))
        self.throttle_us_pub.publish(Int32(data=int(round(throttle_us))))
        self.steer_pub.publish(Float32(data=float(steer_ratio(steer_us - self.sp.trim_us, self.sp))))

    def destroy_node(self):
        try:
            self.out.close(self.sp.center_us + self.sp.trim_us, self.vp.neutral_us)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'종료 중 출력 실패: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DriveNode()
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
