"""USB 카메라 노드 (저지연 버전).

- MJPG 포맷으로 640x480 30fps 요청 (YUYV 는 USB 대역폭 때문에 fps 가 떨어진다)
- 캡처 전용 스레드가 계속 grab 해서 항상 최신 프레임만 발행한다 (버퍼에 쌓인 옛 프레임 X)
- cv2.imshow 없음 (화면 출력은 지연과 CPU 를 잡아먹는다. 확인은 rqt_image_view 로)
- QoS: BEST_EFFORT, depth 1 (구독자가 늦으면 옛 프레임은 버린다)
- publish_compressed:=true 이면 /camera/image_raw/compressed (JPEG) 도 발행 (rosbag 기록용)

발행 토픽
  /camera/image_raw             sensor_msgs/Image (bgr8)
  /camera/image_raw/compressed  sensor_msgs/CompressedImage (옵션)
"""

import threading
import time

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')
        self.device = self.declare_parameter('device', '/dev/video0').value
        self.width = int(self.declare_parameter('width', 640).value)
        self.height = int(self.declare_parameter('height', 480).value)
        self.fps = int(self.declare_parameter('fps', 30).value)
        self.use_mjpg = bool(self.declare_parameter('mjpg', True).value)
        self.frame_id = self.declare_parameter('frame_id', 'camera').value
        self.publish_raw = bool(self.declare_parameter('publish_raw', True).value)
        self.publish_compressed = bool(
            self.declare_parameter('publish_compressed', False).value)
        self.jpeg_quality = int(self.declare_parameter('jpeg_quality', 80).value)
        # 자동 노출이 길어지면 모션 블러가 커진다. 0 이면 건드리지 않음
        self.exposure = int(self.declare_parameter('exposure', 0).value)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.raw_pub = self.create_publisher(Image, '/camera/image_raw', qos)
        self.jpg_pub = self.create_publisher(
            CompressedImage, '/camera/image_raw/compressed', qos)

        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f'카메라 {self.device} 를 열 수 없습니다.')
        if self.use_mjpg:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if self.exposure > 0:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # V4L2: 1 = manual
            self.cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure)

        self.running = True
        self.count = 0
        self.last_report = time.monotonic()
        self.thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.thread.start()
        self.get_logger().info(
            f'카메라 시작: {self.device} {self.width}x{self.height}@{self.fps} '
            f'mjpg={self.use_mjpg} compressed={self.publish_compressed}')

    def capture_loop(self):
        while self.running and rclpy.ok():
            ok, frame = self.cap.read()
            if not ok:
                self.get_logger().warning('카메라 프레임을 읽을 수 없습니다.',
                                          throttle_duration_sec=1.0)
                time.sleep(0.01)
                continue
            stamp = self.get_clock().now().to_msg()
            if self.publish_raw:
                msg = Image()
                msg.header.stamp = stamp
                msg.header.frame_id = self.frame_id
                msg.height, msg.width = frame.shape[:2]
                msg.encoding = 'bgr8'
                msg.is_bigendian = 0
                msg.step = msg.width * 3
                msg.data = frame.tobytes()
                self.raw_pub.publish(msg)
            if self.publish_compressed:
                ok, buf = cv2.imencode('.jpg', frame,
                                       [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                if ok:
                    cmsg = CompressedImage()
                    cmsg.header.stamp = stamp
                    cmsg.header.frame_id = self.frame_id
                    cmsg.format = 'jpeg'
                    cmsg.data = buf.tobytes()
                    self.jpg_pub.publish(cmsg)
            self.count += 1
            now = time.monotonic()
            if now - self.last_report > 5.0:
                self.get_logger().info(
                    f'camera {self.count / (now - self.last_report):.1f} fps')
                self.count = 0
                self.last_report = now

    def destroy_node(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
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
