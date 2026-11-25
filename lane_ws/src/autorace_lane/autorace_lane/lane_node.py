"""차선 노드: /yolo/lane_mask → (왜곡 보정 + BEV LUT) → 좌/우 경계 → 중앙 경로.

입력
  /yolo/lane_mask    sensor_msgs/Image mono8 (값 = 클래스, yolo_ws 참고)
  /yolo/LR_sign      std_msgs/Int8   +1 A(왼쪽) / -1 B(오른쪽) → fork_hold_distance 동안 bias
  /lane/change       std_msgs/Int8   +1 왼쪽 / -1 오른쪽 차선 변경 요청 (objavoid)
  /encoder/distance  std_msgs/Float64 (갈림길 bias 유지 거리 계산)
출력
  /lane/path         nav_msgs/Path    중앙 경로 (base_link, 뒷바퀴 원점)
  /lane/target       std_msgs/Float32MultiArray  (lane_core.T_* 인덱스, README 표 참고)
  /lane/zigzag       std_msgs/UInt8   0 없음 / 1 한쪽 / 2 양쪽
  /lane/debug        sensor_msgs/Image (BEV 그림, publish_debug:=true)
"""

import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Float64, Int8, UInt8

from .ground import GroundModel
from .lane_core import LaneEstimator, LaneParams


class LaneNode(Node):

    def __init__(self):
        super().__init__('lane_node')
        d = self.declare_parameter
        lp = LaneParams()
        for name in lp.__dataclass_fields__:
            setattr(lp, name, type(getattr(lp, name))(d(name, getattr(lp, name)).value))
        self.params = lp
        self.calib_file = d('calib_file', '').value
        self.image_width = int(d('image_width', 640).value)
        self.image_height = int(d('image_height', 480).value)
        self.cam_x = float(d('cam_x', 0.33).value)
        self.cam_h = float(d('cam_h', 0.115).value)
        self.pitch_deg = float(d('pitch_deg', 57.0).value)
        self.hfov_deg = float(d('hfov_deg', 100.0).value)
        self.y_max = float(d('y_max', 1.5).value)
        self.stride = int(d('stride', 2).value)
        self.fork_hold_distance = float(d('fork_hold_distance', 3.0).value)
        self.publish_debug = bool(d('publish_debug', False).value)
        self.frame_id = d('frame_id', 'base_link').value

        self.ground = GroundModel(self.image_width, self.image_height, self.calib_file,
                                  self.cam_x, 0.0, self.cam_h, self.pitch_deg, self.hfov_deg)
        self.lut = None
        self.lut_shape = None
        self.est = LaneEstimator(lp)

        self.distance = 0.0
        self.bias = 0
        self.bias_until = -1.0

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, '/yolo/lane_mask', self.on_mask, qos)
        self.create_subscription(Int8, '/yolo/LR_sign', self.on_lr, 10)
        self.create_subscription(Int8, '/lane/change', self.on_change, 10)
        self.create_subscription(Float64, '/encoder/distance', self.on_distance, 10)
        self.path_pub = self.create_publisher(Path, '/lane/path', 10)
        self.target_pub = self.create_publisher(Float32MultiArray, '/lane/target', 10)
        self.zig_pub = self.create_publisher(UInt8, '/lane/zigzag', 10)
        self.debug_pub = self.create_publisher(Image, '/lane/debug', qos)
        self.t_sum = 0.0
        self.t_n = 0
        src = self.calib_file or f'pinhole h={self.cam_h} pitch={self.pitch_deg} fov={self.hfov_deg}'
        self.get_logger().info(f'lane_node 시작 (ground model: {src})')

    def on_distance(self, msg):
        self.distance = msg.data
        if self.bias != 0 and self.distance > self.bias_until:
            self.get_logger().info('갈림길 bias 해제')
            self.bias = 0

    def on_lr(self, msg):
        b = 1 if msg.data > 0 else -1
        if b != self.bias:
            self.get_logger().info(f'갈림길 표지: {"A(왼쪽)" if b > 0 else "B(오른쪽)"}')
        self.bias = b
        self.bias_until = self.distance + self.fork_hold_distance

    def on_change(self, msg):
        if msg.data != 0:
            self.est.lane_change(1 if msg.data > 0 else -1)
            self.get_logger().info(f'차선 변경 요청 {msg.data:+d}')

    def on_mask(self, msg):
        t0 = time.monotonic()
        mask = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
        if self.lut is None or self.lut_shape != mask.shape:
            p = self.params
            self.lut = self.ground.build_lut(msg.width, msg.height, p.x_min - 0.05,
                                             p.x_max + 0.05, self.y_max)
            self.lut_shape = mask.shape
        s = max(1, self.stride)
        sub = mask[::s, ::s]
        lut = self.lut[::s, ::s]
        vv, uu = np.nonzero(sub)
        xy = lut[vv, uu]
        cls = sub[vv, uu]
        ok = np.isfinite(xy[:, 0])
        xy, cls = xy[ok].astype(np.float64), cls[ok]

        res = self.est.update(xy, cls, self.bias)

        path = Path()
        path.header.stamp = msg.header.stamp
        path.header.frame_id = self.frame_id
        for x, y in res.path:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)
        # 경로를 먼저 보내야 controller 가 target 받을 때 최신 경로를 쓴다
        tgt = Float32MultiArray()
        tgt.data = [float(v) for v in res.target_array(self.bias)]
        self.target_pub.publish(tgt)
        self.zig_pub.publish(UInt8(data=res.zigzag))

        if self.publish_debug:
            self._debug(xy, cls, res, msg.header)
        self.t_sum += time.monotonic() - t0
        self.t_n += 1
        if self.t_n >= 300:
            self.get_logger().info(f'lane 평균 {1000 * self.t_sum / self.t_n:.1f} ms')
            self.t_sum, self.t_n = 0.0, 0

    def _debug(self, xy, cls, res, header):
        """BEV 그림: 위 = 앞, 1px = 1cm."""
        p = self.params
        W, H = 300, int((p.x_max + 0.1) * 100)
        img = np.zeros((H, W, 3), np.uint8)
        colors = np.array([[0, 0, 0], [0, 220, 255], [255, 255, 255], [255, 0, 255],
                           [0, 128, 255], [255, 255, 0], [0, 255, 0], [255, 0, 0]], np.uint8)

        def px(x, y):
            return (H - 1 - (x * 100).astype(int)), (W // 2 - (y * 100).astype(int))

        r, c = px(xy[:, 0], xy[:, 1])
        ok = (r >= 0) & (r < H) & (c >= 0) & (c < W)
        img[r[ok], c[ok]] = colors[np.clip(cls[ok], 0, 7)]
        xs = np.linspace(p.x_min, p.x_max, 40)
        for side, col in ((res.left, (0, 0, 255)), (res.right, (255, 0, 0))):
            if side.poly is not None:
                r, c = px(xs, np.polyval(side.poly, xs))
                ok = (r >= 0) & (r < H) & (c >= 0) & (c < W)
                img[r[ok], c[ok]] = col
        if res.valid:
            r, c = px(res.path[:, 0], res.path[:, 1])
            ok = (r >= 0) & (r < H) & (c >= 0) & (c < W)
            img[r[ok], c[ok]] = (0, 255, 0)
        m = Image()
        m.header = header
        m.height, m.width = img.shape[:2]
        m.encoding = 'bgr8'
        m.step = m.width * 3
        m.data = img.tobytes()
        self.debug_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LaneNode()
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
