#!/usr/bin/env python3
"""Drive the car by hand (keyboard or USB joystick) while watching the
camera, and record everything for labeling and for driving data.

Run with ROS 2 sourced:
  source /opt/ros/jazzy/setup.bash
  python3 drive_record.py

Keys are read from the camera window (click it first so it has focus):
  w / s   : speed +step / -step (below 1680 us is reverse/brake)
  a / d   : steering left / right
  c       : steering center
  space   : stop (ESC neutral 1680 us)
  r       : pause / resume JPEG saving (the rosbag keeps recording)
  p       : save one photo now
  q / ESC : quit (steering centered, throttle neutral, MUX released)

Joystick (/dev/input/js0, found automatically, can be plugged in later):
  right stick left/right : steering   (--joy-steer-axis, default 3)
  left stick up/down     : throttle   (--joy-throttle-axis, default 1)
  button 0 (A / X)       : pause / resume JPEG saving
  Releasing the throttle stick returns to neutral. If the joystick is
  unplugged while driving, the car stops.
  --joy-debug prints axis/button numbers to find the right mapping.

Everything goes into <out>/session_YYYYmmdd_HHMMSS/:
  rosbag/            ros2 bag (mcap), recorded for the whole run:
                       /camera/image_raw/compressed   camera, jpeg, ~30 Hz
                       /bev/image_raw/compressed      BEV, jpeg
                       /drive/steering_us, /drive/throttle_us  (Int32)
                       /drive/steering                (Float32, -1 right .. +1 left)
                       /drive/source                  (String: keyboard / joystick)
                       /encoder/*, /scan              if those nodes are running
                     --bag-raw-image also publishes and records /camera/image_raw
                     (uncompressed, ~1.6 GB/min) for replaying into lane-follow code.
  images/000000.jpg  camera frames for labeling (--record-fps, default 2/s)
  bev/000000.jpg     BEV of the same frame (same number)
  labels.csv         filename, bev_filename, stamp, steering_us, throttle_us,
                     steering, source
The stamp in labels.csv is the header.stamp of the same frame in the rosbag,
so each JPEG can be matched to the bag exactly.
JPEG saving starts on and also saves while throttle is neutral (the car may
be moved by the RC transmitter or by hand); --skip-when-stopped skips those
frames. --no-bag / --no-bev turn those parts off.
To get labeling images out of a rosbag later, use bag_to_images.py.

camera_node must not be running, because this script opens the camera itself
and publishes the camera topic.
"""

import argparse
import csv
import os
import signal
import struct
import subprocess
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

# Hardware values copied from ros2_ws/ujin_versin.py and keyboard_drive.py
# (verified on the car) so this script runs on its own.
I2C_BUS = 7
PCA_ADDR = 0x40
PCA_FREQ = 300
PCA_PRESCALE = int(round(25_000_000.0 / (4096.0 * PCA_FREQ) - 1.0))
PCA_ACTUAL_FREQ = 25_000_000.0 / (4096.0 * (PCA_PRESCALE + 1))
MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06

MUX_PIN = 15
MUX_FREQ = 50
MUX_DUTY = 10.0

# 실차 확인: CH8=조향, CH9=ESC (keyboard_drive.py와 동일)
STEERING_CH = 8
THROTTLE_CH = 9

NEUTRAL_US = 1680  # 실측 정지 (1765 부터 바퀴가 돎, 최고속 1950)
STEERING_CENTER_US = 1660
STEERING_LEFT_US = 2000
STEERING_RIGHT_US = 1400

# Hard bounds so a typo in the options cannot send a full-throttle pulse.
ABSOLUTE_MAX_FORWARD_US = 1950
ABSOLUTE_MIN_REVERSE_US = 1000
DEFAULT_MAX_FORWARD_US = 1850
DEFAULT_MIN_REVERSE_US = 1480
# Full joystick deflection; kept low because the stick jumps straight there.
DEFAULT_JOY_MAX_FORWARD_US = 1810
DEFAULT_JOY_MIN_REVERSE_US = 1580

WINDOW = 'Drive + Record'
BEV_WINDOW = 'BEV'


def bev_matrix(width, height):
    """Perspective matrix for the BEV, copied from ujin_versin.bird_eye_view."""
    source = np.float32([
        (int(width * 0.30), int(height * 0.55)),
        (int(width * 0.70), int(height * 0.55)),
        (width - 1, height - 1),
        (0, height - 1),
    ])
    bev_width = int(width * 0.60)
    x_min = int((width - bev_width) / 2)
    destination = np.float32([
        (x_min, 0),
        (x_min + bev_width, 0),
        (x_min + bev_width, height),
        (x_min, height),
    ])
    return cv2.getPerspectiveTransform(source, destination)


def init_pca9685(bus):
    """Configure the PCA9685 for servo and ESC output at PCA_FREQ."""
    old_mode = bus.read_byte_data(PCA_ADDR, MODE1)
    awake_mode = old_mode & ~0x10
    bus.write_byte_data(PCA_ADDR, MODE1, (awake_mode & 0x7F) | 0x10)
    bus.write_byte_data(PCA_ADDR, PRESCALE, PCA_PRESCALE)
    bus.write_byte_data(PCA_ADDR, MODE1, awake_mode)
    time.sleep(0.005)
    bus.write_byte_data(PCA_ADDR, MODE1, awake_mode | 0xA1)
    time.sleep(0.01)


def set_pwm_us(bus, channel, pulse_us):
    """Write one PCA9685 channel pulse width in microseconds."""
    ticks = int(round(pulse_us * PCA_ACTUAL_FREQ * 4096 / 1_000_000))
    ticks = max(0, min(4095, ticks))
    register = LED0_ON_L + 4 * channel
    values = (0, 0, ticks & 0xFF, (ticks >> 8) & 0x0F)
    for offset, value in enumerate(values):
        bus.write_byte_data(PCA_ADDR, register + offset, value)


class KeyboardHardware:
    """PCA9685 steering/throttle outputs and the MUX SEL pin."""

    def __init__(self, steering_ch, throttle_ch):
        from smbus2 import SMBus
        import Jetson.GPIO as GPIO

        self.steering_ch = steering_ch
        self.throttle_ch = throttle_ch
        self.bus = None
        self.gpio = GPIO
        self.mux_pwm = None
        try:
            self.bus = SMBus(I2C_BUS)
            try:
                init_pca9685(self.bus)
            except OSError as exc:
                raise SystemExit(
                    f'PCA9685가 I2C 버스 {I2C_BUS}, 주소 0x{PCA_ADDR:02x}에서 응답하지 '
                    f'않습니다({exc}). VCC/SDA/SCL/GND 배선을 확인하고 '
                    f'"i2cdetect -y -r {I2C_BUS}"에서 40이 보이는지 확인하세요. '
                    '모터 없이 시험하려면 --dry-run을 쓰세요.') from exc
            self.set_speed(NEUTRAL_US)
            self.set_steering(STEERING_CENTER_US)

            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(MUX_PIN, GPIO.OUT)
            self.mux_pwm = GPIO.PWM(MUX_PIN, MUX_FREQ)
            self.mux_pwm.start(MUX_DUTY)
        except BaseException:
            self.close()
            raise

    def set_speed(self, pulse_us):
        set_pwm_us(self.bus, self.throttle_ch, pulse_us)

    def set_steering(self, pulse_us):
        set_pwm_us(self.bus, self.steering_ch, pulse_us)

    def close(self):
        if self.bus is not None:
            try:
                set_pwm_us(self.bus, self.throttle_ch, NEUTRAL_US)
                set_pwm_us(self.bus, self.steering_ch, STEERING_CENTER_US)
                time.sleep(0.2)
            except OSError as exc:
                print(f'안전 출력 실패: {exc}')
        if self.mux_pwm is not None:
            self.mux_pwm.stop()
            self.mux_pwm = None
        if self.gpio is not None:
            self.gpio.cleanup()
            self.gpio = None
        if self.bus is not None:
            self.bus.close()
            self.bus = None


class DryRunHardware:
    """Ignore outputs instead of touching I2C/GPIO."""

    def set_speed(self, pulse_us):
        pass

    def set_steering(self, pulse_us):
        pass

    def close(self):
        pass


def steering_ratio(steering_us):
    """-1 = full right, 0 = center, +1 = full left (left is the larger pulse)."""
    if steering_us >= STEERING_CENTER_US:
        return (steering_us - STEERING_CENTER_US) / (STEERING_LEFT_US - STEERING_CENTER_US)
    return -(STEERING_CENTER_US - steering_us) / (STEERING_CENTER_US - STEERING_RIGHT_US)


def ratio_to_steering_us(ratio):
    """Inverse of steering_ratio: -1 right .. +1 left -> pulse in us."""
    ratio = max(-1.0, min(1.0, ratio))
    if ratio >= 0:
        return int(round(STEERING_CENTER_US + ratio * (STEERING_LEFT_US - STEERING_CENTER_US)))
    return int(round(STEERING_CENTER_US + ratio * (STEERING_CENTER_US - STEERING_RIGHT_US)))


def ratio_to_throttle_us(ratio, min_reverse, max_forward):
    """-1 full reverse .. +1 full forward -> ESC pulse in us."""
    ratio = max(-1.0, min(1.0, ratio))
    if ratio >= 0:
        return int(round(NEUTRAL_US + ratio * (max_forward - NEUTRAL_US)))
    return int(round(NEUTRAL_US + ratio * (NEUTRAL_US - min_reverse)))


class Joystick:
    """Read a Linux joystick device (/dev/input/jsN) without extra packages.

    The device is opened lazily and reopened once a second, so it can be
    plugged in after start. Axis values are stored as -1.0 .. +1.0.
    """

    EVENT = struct.Struct('IhBB')  # time ms, value, type, number
    TYPE_BUTTON = 0x01
    TYPE_AXIS = 0x02
    TYPE_INIT = 0x80

    def __init__(self, path, debug=False):
        self.path = path
        self.debug = debug
        self.fd = None
        self.axes = {}
        self.last_try = 0.0

    @property
    def connected(self):
        return self.fd is not None

    def poll(self):
        """Read pending events. Returns (moved_axes, pressed_buttons)."""
        moved, pressed = set(), []
        if self.fd is None:
            now = time.monotonic()
            if now - self.last_try < 1.0:
                return moved, pressed
            self.last_try = now
            try:
                self.fd = os.open(self.path, os.O_RDONLY | os.O_NONBLOCK)
            except OSError:
                return moved, pressed
            print(f'조이스틱 연결됨: {self.path}')
        while True:
            try:
                data = os.read(self.fd, self.EVENT.size * 64)
            except BlockingIOError:
                break
            except OSError:
                print('조이스틱 연결 끊김')
                self.close()
                break
            if not data:
                break
            for offset in range(0, len(data) - self.EVENT.size + 1, self.EVENT.size):
                _, value, etype, number = self.EVENT.unpack_from(data, offset)
                if etype & self.TYPE_AXIS:
                    self.axes[number] = value / 32767.0
                    if not etype & self.TYPE_INIT:
                        moved.add(number)
                        if self.debug:
                            print(f'axis {number} = {self.axes[number]:+.2f}')
                elif etype & self.TYPE_BUTTON and value and not etype & self.TYPE_INIT:
                    pressed.append(number)
                    if self.debug:
                        print(f'button {number}')
        return moved, pressed

    def axis(self, number, deadzone):
        value = self.axes.get(number, 0.0)
        return 0.0 if abs(value) < deadzone else value

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
        self.axes.clear()


def split_stamp(stamp_ns):
    return stamp_ns // 1_000_000_000, stamp_ns % 1_000_000_000


class RosOutput:
    """Publish camera, BEV and drive commands, and record them with ros2 bag."""

    def __init__(self, session, raw_image, bev, extra_topics, jpeg_quality):
        try:
            import rclpy
            from sensor_msgs.msg import CompressedImage, Image
            from std_msgs.msg import Float32, Int32, String
        except ImportError as exc:
            raise SystemExit('ROS 2를 찾을 수 없습니다. 먼저 '
                             '"source /opt/ros/jazzy/setup.bash"를 실행하거나 '
                             '--no-bag으로 실행하세요.') from exc
        self.rclpy = rclpy
        self.msgs = (CompressedImage, Image, Float32, Int32, String)
        self.params = [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality]
        self.proc = None
        self.log = None
        rclpy.init()
        self.node = rclpy.create_node('drive_record')
        create = self.node.create_publisher
        self.camera_pub = create(CompressedImage, '/camera/image_raw/compressed', 10)
        self.raw_pub = create(Image, '/camera/image_raw', 10) if raw_image else None
        self.bev_pub = create(CompressedImage, '/bev/image_raw/compressed', 10) if bev else None
        self.steering_us_pub = create(Int32, '/drive/steering_us', 10)
        self.throttle_us_pub = create(Int32, '/drive/throttle_us', 10)
        self.steering_pub = create(Float32, '/drive/steering', 10)
        self.source_pub = create(String, '/drive/source', 10)

        topics = ['/camera/image_raw/compressed', '/drive/steering_us',
                  '/drive/throttle_us', '/drive/steering', '/drive/source',
                  '/encoder/count', '/encoder/pulses', '/encoder/direction',
                  '/encoder/frequency', '/encoder/period_ms', '/scan']
        if raw_image:
            topics.append('/camera/image_raw')
        if bev:
            topics.append('/bev/image_raw/compressed')
        topics += extra_topics

        self.bag_path = session / 'rosbag'
        self.log = open(session / 'rosbag.log', 'w')
        try:
            # Own session so the terminal's Ctrl+C does not reach ros2 bag;
            # close() stops it exactly once so the mcap file is finalized.
            self.proc = subprocess.Popen(
                ['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(self.bag_path), *topics],
                stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
        except FileNotFoundError as exc:
            self.close()
            raise SystemExit('ros2 명령을 찾을 수 없습니다. '
                             '"source /opt/ros/jazzy/setup.bash" 후 실행하세요.') from exc
        # Wait until the recorder subscribed, so the first frames are not lost.
        deadline = time.monotonic() + 10.0
        while self.camera_pub.get_subscription_count() == 0:
            if self.proc.poll() is not None:
                self.close()
                raise SystemExit(f'ros2 bag record가 종료됐습니다. {session / "rosbag.log"}를 '
                                 '확인하세요.')
            if time.monotonic() > deadline:
                print('경고: rosbag 구독 확인이 늦어지고 있습니다. 계속 진행합니다.')
                break
            time.sleep(0.1)
        print(f'rosbag 녹화 시작: {self.bag_path}')

    def _compressed(self, image, stamp, frame_id):
        msg = self.msgs[0]()
        msg.header.stamp.sec, msg.header.stamp.nanosec = stamp
        msg.header.frame_id = frame_id
        msg.format = 'jpeg'
        ok, buf = cv2.imencode('.jpg', image, self.params)
        if not ok:
            raise OSError('JPEG 인코딩 실패')
        msg.data = buf.tobytes()
        return msg

    def publish(self, frame, bev, stamp_ns, steering_us, throttle_us, source):
        CompressedImage, Image, Float32, Int32, String = self.msgs
        stamp = split_stamp(stamp_ns)
        # Commands first, so a reader of the bag already has them at each frame.
        self.steering_us_pub.publish(Int32(data=int(steering_us)))
        self.throttle_us_pub.publish(Int32(data=int(throttle_us)))
        self.steering_pub.publish(Float32(data=float(steering_ratio(steering_us))))
        self.source_pub.publish(String(data=source))
        self.camera_pub.publish(self._compressed(frame, stamp, 'camera'))
        if self.raw_pub is not None:
            msg = Image()
            msg.header.stamp.sec, msg.header.stamp.nanosec = stamp
            msg.header.frame_id = 'camera'
            msg.height, msg.width = frame.shape[:2]
            msg.encoding = 'bgr8'
            msg.step = frame.shape[1] * 3
            msg.data = frame.tobytes()
            self.raw_pub.publish(msg)
        if self.bev_pub is not None and bev is not None:
            self.bev_pub.publish(self._compressed(bev, stamp, 'bev'))

    @property
    def bag_running(self):
        return self.proc is not None and self.proc.poll() is None

    def close(self):
        if self.proc is not None:
            if self.proc.poll() is None:
                self.proc.send_signal(signal.SIGINT)
                try:
                    self.proc.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    self.proc.terminate()
                    self.proc.wait(timeout=5)
            print(f'rosbag 저장 완료: {self.bag_path}')
            self.proc = None
        if self.log is not None:
            self.log.close()
            self.log = None
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
            self.rclpy.try_shutdown()


class Recorder:
    """Write JPEG frames and a labels.csv row for each one."""

    def __init__(self, out_dir, jpeg_quality, save_bev):
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.session = Path(out_dir).expanduser() / f'session_{stamp}'
        self.images = self.session / 'images'
        self.images.mkdir(parents=True, exist_ok=True)
        self.bev_images = self.session / 'bev' if save_bev else None
        if self.bev_images is not None:
            self.bev_images.mkdir()
        self.params = [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality]
        self.csv_file = open(self.session / 'labels.csv', 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['filename', 'bev_filename', 'stamp', 'steering_us',
                              'throttle_us', 'steering', 'source'])
        self.count = 0

    def _write(self, path, image):
        if not cv2.imwrite(str(path), image, self.params):
            raise OSError(f'이미지 저장 실패: {path}')

    def save(self, frame, bev, stamp_ns, steering_us, throttle_us, source):
        name = f'{self.count:06d}.jpg'
        self._write(self.images / name, frame)
        bev_name = ''
        if self.bev_images is not None:
            self._write(self.bev_images / name, bev)
            bev_name = f'bev/{name}'
        sec, nsec = split_stamp(stamp_ns)
        self.writer.writerow([f'images/{name}', bev_name, f'{sec}.{nsec:09d}',
                              steering_us, throttle_us,
                              f'{steering_ratio(steering_us):.3f}', source])
        self.csv_file.flush()
        self.count += 1

    def close(self):
        self.csv_file.close()


def draw_overlay(frame, speed, steering, recording, saved, fps, bag, source, joystick):
    view = frame.copy()
    if speed > NEUTRAL_US:
        state = 'FWD'
    elif speed < NEUTRAL_US:
        state = 'REV'
    else:
        state = 'STOP'
    joy = 'on' if joystick.connected else 'off'
    lines = [
        f'{state} throttle={speed}us  steer={steering}us ({steering_ratio(steering):+.2f})',
        f'jpg={saved}  bag={bag}  input={source}  joy={joy}  fps={fps:.1f}',
        'w/s speed  a/d steer  c center  space stop  r jpg on/off  p photo  q quit',
    ]
    for i, text in enumerate(lines):
        y = 22 + i * 22
        cv2.putText(view, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
        cv2.putText(view, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    if recording:
        h, w = view.shape[:2]
        cv2.circle(view, (w - 24, 24), 10, (0, 0, 255), -1)
        cv2.putText(view, 'REC', (w - 76, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    # Steering bar at the bottom: marker moves left/right with the wheels.
    h, w = view.shape[:2]
    cx = w // 2
    cv2.line(view, (cx - 100, h - 20), (cx + 100, h - 20), (200, 200, 200), 2)
    marker = int(cx - steering_ratio(steering) * 100)
    cv2.circle(view, (marker, h - 20), 7, (0, 255, 255), -1)
    return view


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--camera', default='/dev/video0', help='카메라 장치(기본 /dev/video0)')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--out', default='~/datasets',
                        help='저장 폴더(기본 ~/datasets, 실행마다 session_* 생성)')
    parser.add_argument('--record-fps', type=float, default=2.0,
                        help='JPEG 초당 저장 장수(기본 2)')
    parser.add_argument('--skip-when-stopped', action='store_true',
                        help=f'속도가 중립({NEUTRAL_US} us)일 때는 JPEG 저장 안 함')
    parser.add_argument('--jpeg-quality', type=int, default=95)
    parser.add_argument('--no-bev', action='store_true',
                        help='BEV 창과 BEV 저장 끄기')
    parser.add_argument('--no-bag', action='store_true',
                        help='rosbag 녹화와 ROS 토픽 발행 끄기')
    parser.add_argument('--bag-raw-image', action='store_true',
                        help='/camera/image_raw(무압축)도 발행·녹화(용량 큼)')
    parser.add_argument('--bag-jpeg-quality', type=int, default=85,
                        help='rosbag 압축 이미지 품질(기본 85)')
    parser.add_argument('--bag-extra-topics', nargs='*', default=[],
                        help='rosbag에 추가로 녹화할 토픽')
    parser.add_argument('--joystick', default='/dev/input/js0',
                        help='조이스틱 장치(기본 /dev/input/js0)')
    parser.add_argument('--joy-steer-axis', type=int, default=3)
    parser.add_argument('--joy-throttle-axis', type=int, default=1)
    parser.add_argument('--joy-invert-steer', action='store_true')
    parser.add_argument('--joy-invert-throttle', action='store_true')
    parser.add_argument('--joy-record-button', type=int, default=0)
    parser.add_argument('--joy-deadzone', type=float, default=0.08)
    parser.add_argument('--joy-max-forward-us', type=int, default=DEFAULT_JOY_MAX_FORWARD_US,
                        help=f'스틱 끝까지 올렸을 때 전진(기본 {DEFAULT_JOY_MAX_FORWARD_US})')
    parser.add_argument('--joy-min-reverse-us', type=int, default=DEFAULT_JOY_MIN_REVERSE_US,
                        help=f'스틱 끝까지 내렸을 때 후진(기본 {DEFAULT_JOY_MIN_REVERSE_US})')
    parser.add_argument('--joy-debug', action='store_true',
                        help='조이스틱 축/버튼 번호 출력')
    parser.add_argument('--steering-ch', type=int, default=STEERING_CH)
    parser.add_argument('--throttle-ch', type=int, default=THROTTLE_CH)
    parser.add_argument('--dry-run', action='store_true',
                        help='PWM 출력 없이 카메라/저장만 확인')
    parser.add_argument('--speed-step', type=int, default=5,
                        help='w/s 한 번에 바뀌는 속도 펄스(us, 기본 5)')
    parser.add_argument('--steer-step', type=int, default=40,
                        help='a/d 한 번에 바뀌는 조향 펄스(us, 기본 40)')
    parser.add_argument('--max-forward-us', type=int, default=DEFAULT_MAX_FORWARD_US)
    parser.add_argument('--min-reverse-us', type=int, default=DEFAULT_MIN_REVERSE_US)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    max_forward = min(args.max_forward_us, ABSOLUTE_MAX_FORWARD_US)
    min_reverse = max(args.min_reverse_us, ABSOLUTE_MIN_REVERSE_US)
    if not min_reverse <= NEUTRAL_US <= max_forward:
        raise SystemExit(f'후진 하한 <= {NEUTRAL_US} <= 전진 상한이어야 합니다')
    joy_max_forward = min(args.joy_max_forward_us, max_forward)
    joy_min_reverse = max(args.joy_min_reverse_us, min_reverse)
    if not joy_min_reverse <= NEUTRAL_US <= joy_max_forward:
        raise SystemExit(f'조이스틱 후진 하한 <= {NEUTRAL_US} <= 조이스틱 전진 상한이어야 합니다')
    if args.steering_ch == args.throttle_ch:
        raise SystemExit('조향 채널과 ESC 채널이 같으면 안 됩니다')
    steer_min = min(STEERING_LEFT_US, STEERING_RIGHT_US)
    steer_max = max(STEERING_LEFT_US, STEERING_RIGHT_US)
    left_sign = 1 if STEERING_LEFT_US > STEERING_RIGHT_US else -1
    steer_sign = 1 if args.joy_invert_steer else -1      # stick right -> steer right
    throttle_sign = 1 if args.joy_invert_throttle else -1  # stick up is negative

    cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(f'카메라 {args.camera}를 열 수 없습니다. '
                         'camera_node가 실행 중이면 먼저 종료하세요.')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, 30)

    print(__doc__)
    print(f'채널: 조향 CH{args.steering_ch}, ESC CH{args.throttle_ch}')
    print(f'속도 범위: 후진 {min_reverse} ~ 중립 {NEUTRAL_US} ~ 전진 {max_forward} us '
          f'(조이스틱 {joy_min_reverse} ~ {joy_max_forward} us)')
    print('처음에는 바퀴를 띄운 상태에서 시험하세요.\n')

    # Ctrl+C, kill, or closing the terminal all go through the same cleanup
    # (stop the car, finalize the rosbag), even when SIGINT was inherited
    # as ignored (e.g. started in the background).
    def request_stop(signum, frame):
        raise KeyboardInterrupt

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, request_stop)

    hardware = None
    recorder = None
    ros = None
    joystick = Joystick(args.joystick, args.joy_debug)
    try:
        hardware = DryRunHardware() if args.dry_run else KeyboardHardware(
            args.steering_ch, args.throttle_ch)
        recorder = Recorder(args.out, args.jpeg_quality, not args.no_bev)
        print(f'저장 위치: {recorder.session}')
        if not args.no_bag:
            ros = RosOutput(recorder.session, args.bag_raw_image, not args.no_bev,
                            args.bag_extra_topics, args.bag_jpeg_quality)
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        bev_size = None
        bev_mat = None

        speed = NEUTRAL_US
        steering = STEERING_CENTER_US
        applied = (speed, steering)
        hardware.set_speed(speed)
        hardware.set_steering(steering)
        source = 'keyboard'
        recording = True
        record_interval = 1.0 / args.record_fps
        last_saved = 0.0
        fps = 0.0
        last_frame = time.monotonic()

        while True:
            ok, frame = cap.read()
            if not ok:
                # Stop the car rather than driving blind.
                hardware.set_speed(NEUTRAL_US)
                raise SystemExit('카메라 프레임을 읽을 수 없어 정지합니다.')
            stamp_ns = time.time_ns()
            now = time.monotonic()
            fps = 0.9 * fps + 0.1 / max(now - last_frame, 1e-3)
            last_frame = now

            bev = None
            if not args.no_bev:
                h, w = frame.shape[:2]
                if bev_size != (w, h):
                    bev_size = (w, h)
                    bev_mat = bev_matrix(w, h)
                bev = cv2.warpPerspective(frame, bev_mat, bev_size)

            # Joystick: sticks take over as soon as they move.
            was_connected = joystick.connected
            moved, pressed = joystick.poll()
            if was_connected and not joystick.connected and source == 'joystick':
                speed, steering = NEUTRAL_US, STEERING_CENTER_US
            if moved & {args.joy_steer_axis, args.joy_throttle_axis}:
                source = 'joystick'
            if source == 'joystick' and joystick.connected:
                steering = ratio_to_steering_us(
                    steer_sign * joystick.axis(args.joy_steer_axis, args.joy_deadzone))
                speed = ratio_to_throttle_us(
                    throttle_sign * joystick.axis(args.joy_throttle_axis, args.joy_deadzone),
                    joy_min_reverse, joy_max_forward)
            if args.joy_record_button in pressed:
                recording = not recording
                print(f'JPEG 저장 {"재개" if recording else "일시정지"} (저장 {recorder.count}장)')

            if ros is not None:
                ros.publish(frame, bev, stamp_ns, steering, speed, source)
            moving = speed != NEUTRAL_US
            if (recording and now - last_saved >= record_interval
                    and (moving or not args.skip_when_stopped)):
                recorder.save(frame, bev, stamp_ns, steering, speed, source)
                last_saved = now

            if ros is None:
                bag = 'off'
            else:
                bag = 'REC' if ros.bag_running else 'STOPPED'
            cv2.imshow(WINDOW, draw_overlay(frame, speed, steering, recording,
                                            recorder.count, fps, bag, source, joystick))
            if bev is not None:
                cv2.imshow(BEV_WINDOW, bev)
            # Quit with q/ESC/Ctrl+C only: this OpenCV build reports -1 for
            # WND_PROP_VISIBLE, so closing the window cannot be detected.
            key = cv2.waitKey(1) & 0xFF
            if key != 255:
                ch = chr(key).lower()
                if ch == 'q' or key == 27:
                    break
                if ch in 'wsadc ':
                    source = 'keyboard'
                if ch == 'w':
                    speed = min(max_forward, speed + args.speed_step)
                elif ch == 's':
                    speed = max(min_reverse, speed - args.speed_step)
                elif ch == ' ':
                    speed = NEUTRAL_US
                elif ch == 'a':
                    steering += left_sign * args.steer_step
                elif ch == 'd':
                    steering -= left_sign * args.steer_step
                elif ch == 'c':
                    steering = STEERING_CENTER_US
                elif ch == 'r':
                    recording = not recording
                    print(f'JPEG 저장 {"재개" if recording else "일시정지"} '
                          f'(저장 {recorder.count}장)')
                elif ch == 'p':
                    recorder.save(frame, bev, stamp_ns, steering, speed, source)
                    print(f'사진 저장 ({recorder.count}장)')
                steering = min(steer_max, max(steer_min, steering))

            if (speed, steering) != applied:
                hardware.set_speed(speed)
                hardware.set_steering(steering)
                applied = (speed, steering)
    except KeyboardInterrupt:
        pass
    finally:
        # A second Ctrl+C must not cut the cleanup short.
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, signal.SIG_IGN)
        print('\n정지 및 조향 중앙으로 복귀합니다.')
        if hardware is not None:
            hardware.close()
        cap.release()
        cv2.destroyAllWindows()
        joystick.close()
        if ros is not None:
            ros.close()
        if recorder is not None:
            recorder.close()
            print(f'JPEG 총 {recorder.count}장 저장: {recorder.session}')


if __name__ == '__main__':
    main()
