"""YOLO-seg (TensorRT FP16) 하나로 차선 마스크와 표지판/신호등을 동시에 검출한다.

입력
  /camera/image_raw          sensor_msgs/Image (bgr8), BEST_EFFORT depth 1 → 항상 최신 프레임
출력
  /yolo/lane_mask            sensor_msgs/Image mono8 (값 = classes.LANE_IDS, 0 = 배경)
  /yolo/detections           std_msgs/Float32MultiArray [class_idx, conf, x1, y1, x2, y2] * N
                             (좌표는 0~1 정규화, class_idx 는 classes.SIGN_NAMES 의 인덱스)
  /yolo/slow_sign            std_msgs/Bool  (매 프레임, 연속 confirm_frames 프레임 확인)
  /yolo/parking_sign         std_msgs/Bool
  /yolo/no_speed_limit       std_msgs/Bool
  /yolo/cone                 std_msgs/Bool
  /yolo/fixed_obstacle       std_msgs/Bool
  /yolo/LR_sign              std_msgs/Int8  +1 = A(왼쪽), -1 = B(오른쪽). 확인됐을 때만 발행
  /yolo/signal               std_msgs/Int8  0 = 빨강, 1 = 노랑, 2 = 초록. 확인됐을 때만 발행
  /yolo/debug/compressed     sensor_msgs/CompressedImage (publish_debug:=true 일 때만)

모델 준비 (Jetson 에서 한 번): yolo_ws/README.md 참고
  yolo export model=best.pt format=engine half=True imgsz=640 device=0
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Float32MultiArray, Int8

from .classes import LANE_IDS, SIGN_NAMES, normalize
from .core import Confirmer, best_boxes, draw_lane_mask


def _fill(mask, pts, value):
    cv2.fillPoly(mask, [pts], int(value))


class YoloNode(Node):

    def __init__(self):
        super().__init__('yolo_node')
        p = self.declare_parameter
        self.model_path = p('model_path', '').value
        self.imgsz = int(p('imgsz', 640).value)
        self.half = bool(p('half', True).value)
        self.device = p('device', '0').value
        self.lane_conf = float(p('lane_conf', 0.25).value)
        self.sign_conf = float(p('sign_conf', 0.5).value)
        self.mask_width = int(p('mask_width', 320).value)
        self.mask_height = int(p('mask_height', 240).value)
        self.confirm_frames = int(p('confirm_frames', 2).value)
        self.min_area_default = float(p('min_box_area', 0.0005).value)
        self.min_area_light = float(p('min_light_area', 0.0003).value)
        self.publish_debug = bool(p('publish_debug', False).value)
        self.debug_every = int(p('debug_every', 3).value)
        if not self.model_path:
            raise RuntimeError('model_path 파라미터에 best.engine (또는 best.pt) 경로를 넣으세요.')

        from ultralytics import YOLO  # 무거워서 여기서 import
        self.model = YOLO(self.model_path, task='segment')
        self.names = {}
        self._warmup()

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.on_image, qos)
        self.mask_pub = self.create_publisher(Image, '/yolo/lane_mask', qos)
        self.det_pub = self.create_publisher(Float32MultiArray, '/yolo/detections', 10)
        self.bool_pubs = {
            'slow': self.create_publisher(Bool, '/yolo/slow_sign', 10),
            'parking': self.create_publisher(Bool, '/yolo/parking_sign', 10),
            'no_speed_limit': self.create_publisher(Bool, '/yolo/no_speed_limit', 10),
            'cone': self.create_publisher(Bool, '/yolo/cone', 10),
            'fixed_obstacle': self.create_publisher(Bool, '/yolo/fixed_obstacle', 10),
        }
        self.lr_pub = self.create_publisher(Int8, '/yolo/LR_sign', 10)
        self.signal_pub = self.create_publisher(Int8, '/yolo/signal', 10)
        self.debug_pub = self.create_publisher(CompressedImage, '/yolo/debug/compressed', qos)
        self.confirmer = Confirmer(SIGN_NAMES, self.confirm_frames)
        self.frame_no = 0
        self.t_sum = 0.0
        self.t_count = 0
        self.get_logger().info(f'YOLO 시작: {self.model_path}, classes={self.names}')

    def _warmup(self):
        dummy = np.zeros((480, 640, 3), np.uint8)
        for _ in range(3):
            res = self._predict(dummy)
        self.names = {int(k): normalize(v) for k, v in res.names.items()}

    def _predict(self, frame):
        return self.model.predict(
            frame, imgsz=self.imgsz, conf=min(self.lane_conf, self.sign_conf),
            half=self.half, device=self.device, verbose=False, retina_masks=False)[0]

    def on_image(self, msg):
        t0 = time.monotonic()
        if msg.encoding not in ('bgr8', 'rgb8'):
            self.get_logger().error(f'지원하지 않는 encoding {msg.encoding}',
                                    throttle_duration_sec=5.0)
            return
        frame = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            frame = frame[:, :, ::-1]
        res = self._predict(frame)

        polygons = []
        dets = []
        if res.boxes is not None and len(res.boxes):
            cls = res.boxes.cls.int().tolist()
            conf = res.boxes.conf.tolist()
            xyxyn = res.boxes.xyxyn.tolist()
            polys = res.masks.xyn if res.masks is not None else [None] * len(cls)
            for c, s, box, poly in zip(cls, conf, xyxyn, polys):
                name = self.names.get(c, str(c))
                if name in LANE_IDS:
                    if s >= self.lane_conf:
                        polygons.append((name, poly))
                elif s >= self.sign_conf:
                    dets.append((name, s, *box))

        # 1) 차선 마스크
        mask = draw_lane_mask(polygons, self.mask_width, self.mask_height, _fill)
        out = Image()
        out.header = msg.header
        out.height, out.width = mask.shape
        out.encoding = 'mono8'
        out.step = out.width
        out.data = mask.tobytes()
        self.mask_pub.publish(out)

        # 2) 박스
        arr = Float32MultiArray()
        for name, s, x1, y1, x2, y2 in dets:
            if name in SIGN_NAMES:
                arr.data.extend([float(SIGN_NAMES.index(name)), s, x1, y1, x2, y2])
        self.det_pub.publish(arr)

        min_area = {'default': self.min_area_default, 'red_light': self.min_area_light,
                    'yellow_light': self.min_area_light, 'green_light': self.min_area_light}
        best = best_boxes(dets, min_area)
        confirmed = self.confirmer.update(set(best))
        for name, pub in self.bool_pubs.items():
            pub.publish(Bool(data=confirmed[name]))
        # 갈림길: 둘 다 보이면 더 큰 표지
        lr = [(n, best[n]) for n in ('go_a', 'go_b') if confirmed[n]]
        if lr:
            name = max(lr, key=lambda t: (t[1][4] - t[1][2]) * (t[1][5] - t[1][3]))[0]
            self.lr_pub.publish(Int8(data=1 if name == 'go_a' else -1))
        # 신호등: 빨강 > 노랑 > 초록 (애매하면 서는 쪽)
        for name, code in (('red_light', 0), ('yellow_light', 1), ('green_light', 2)):
            if confirmed[name]:
                self.signal_pub.publish(Int8(data=code))
                break

        self.frame_no += 1
        if self.publish_debug and self.frame_no % max(1, self.debug_every) == 0:
            self._publish_debug(frame, mask, dets, msg.header)

        self.t_sum += time.monotonic() - t0
        self.t_count += 1
        if self.t_count >= 150:
            self.get_logger().info(f'YOLO 평균 {1000 * self.t_sum / self.t_count:.1f} ms/frame')
            self.t_sum = 0.0
            self.t_count = 0

    def _publish_debug(self, frame, mask, dets, header):
        img = frame.copy()
        colors = np.array([[0, 0, 0], [0, 220, 255], [255, 255, 255], [255, 0, 255],
                           [0, 128, 255], [255, 255, 0], [0, 255, 0], [255, 0, 0]], np.uint8)
        big = cv2.resize(mask, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
        overlay = colors[np.clip(big, 0, len(colors) - 1)]
        img = np.where(big[..., None] > 0, (img * 0.4 + overlay * 0.6).astype(np.uint8), img)
        h, w = img.shape[:2]
        for name, s, x1, y1, x2, y2 in dets:
            p1 = (int(x1 * w), int(y1 * h))
            cv2.rectangle(img, p1, (int(x2 * w), int(y2 * h)), (0, 0, 255), 2)
            cv2.putText(img, f'{name} {s:.2f}', (p1[0], max(12, p1[1] - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            m = CompressedImage()
            m.header = header
            m.format = 'jpeg'
            m.data = buf.tobytes()
            self.debug_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloNode()
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
