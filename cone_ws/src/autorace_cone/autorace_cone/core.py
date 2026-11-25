"""라바콘 구간: 차선 무시, LiDAR 콘 군집 사이(안쪽 경로)로 주행.

1) ROI 안 LiDAR 점을 각도 순으로 묶어 군집 (gap > cluster_gap 이면 끊음), 너무 큰 군집(벽, LIMO) 제외
2) 직전 목표 경로(y_prev)를 기준으로 콘을 왼쪽/오른쪽으로 나눔
3) 앞쪽 lookahead 안의 가장 가까운 왼쪽·오른쪽 콘의 중점이 목표점.
   한쪽만 보이면 그 콘에서 gap/2 만큼 안쪽
4) 목표점에 Pure Pursuit
활성: YOLO cone 확인 후 LiDAR 콘이 min_cones 개 이상. exit_distance 동안 콘이 없으면 종료
"""

import math

import numpy as np


def scan_to_points(ranges, angle_min, angle_inc, range_min=0.05, range_max=3.0,
                   lidar_x=0.20, lidar_yaw=0.0, flip=False):
    """LaserScan → 차량 좌표 (뒷바퀴 원점) (N,2)."""
    r = np.asarray(ranges, np.float64)
    a = angle_min + angle_inc * np.arange(len(r))
    if flip:
        a = -a
    a = a + lidar_yaw
    ok = np.isfinite(r) & (r > range_min) & (r < range_max)
    r, a = r[ok], a[ok]
    return np.column_stack([lidar_x + r * np.cos(a), r * np.sin(a)])


def clusters(pts, cluster_gap=0.08, max_size=0.18, min_points=2):
    """pts 는 각도 순서 → 인접 거리로 분할. 반환: 콘 중심 (M,2)."""
    if len(pts) == 0:
        return np.zeros((0, 2))
    ang = np.arctan2(pts[:, 1], pts[:, 0])
    pts = pts[np.argsort(ang)]
    d = np.hypot(*np.diff(pts, axis=0).T)
    breaks = np.nonzero(d > cluster_gap)[0] + 1
    centers = []
    for grp in np.split(pts, breaks):
        if len(grp) < min_points:
            continue
        size = np.hypot(*(grp.max(axis=0) - grp.min(axis=0)))
        if size > max_size:
            continue
        centers.append(grp.mean(axis=0))
    return np.array(centers) if centers else np.zeros((0, 2))


class ConeCourse:

    def __init__(self, wheelbase=0.26, roi_x=(0.05, 1.6), roi_y=1.0, lookahead=0.8,
                 half_gap=0.30, min_cones=2, exit_distance=0.6, speed=0.5, max_steer=0.349,
                 steer_filter=0.5, arm_distance=3.0, require_yolo=True):
        self.L = wheelbase
        self.roi_x = roi_x
        self.roi_y = roi_y
        self.lookahead = lookahead
        self.half_gap = half_gap
        self.min_cones = min_cones
        self.exit_distance = exit_distance
        self.speed = speed
        self.max_steer = max_steer
        self.steer_filter = steer_filter
        self.arm_distance = arm_distance
        self.require_yolo = require_yolo
        self.active = False
        self.armed_until = -1e9
        self.last_seen_s = -1e9
        self.y_prev = 0.0
        self.steer = 0.0
        self.target = None

    def on_yolo(self, s):
        self.armed_until = s + self.arm_distance

    def step(self, s, cones):
        """cones: (M,2) 콘 중심. 반환 (speed, steer) 또는 None."""
        if len(cones):
            m = (cones[:, 0] > self.roi_x[0]) & (cones[:, 0] < self.roi_x[1]) & \
                (np.abs(cones[:, 1]) < self.roi_y)
            cones = cones[m]
        if not self.active:
            armed = s < self.armed_until or not self.require_yolo
            if armed and len(cones) >= self.min_cones:
                self.active = True
                self.last_seen_s = s
                self.y_prev = 0.0
            else:
                return None
        if len(cones):
            self.last_seen_s = s
        elif s - self.last_seen_s > self.exit_distance:
            self.active = False
            return None

        ahead = cones[cones[:, 0] < self.roi_x[0] + self.lookahead + 0.4] if len(cones) else cones
        left = ahead[ahead[:, 1] > self.y_prev] if len(ahead) else ahead
        right = ahead[ahead[:, 1] <= self.y_prev] if len(ahead) else ahead
        nl = left[np.argmin(left[:, 0])] if len(left) else None
        nr = right[np.argmin(right[:, 0])] if len(right) else None
        if nl is not None and nr is not None:
            tgt = (nl + nr) / 2.0
        elif nl is not None:
            tgt = nl - np.array([0.0, self.half_gap])
        elif nr is not None:
            tgt = nr + np.array([0.0, self.half_gap])
        else:
            tgt = None
        if tgt is not None:
            tgt = np.array([max(tgt[0], 0.35), tgt[1]])
            self.target = tgt
            self.y_prev = 0.7 * self.y_prev + 0.3 * float(tgt[1])
            ld = math.hypot(*tgt)
            delta = math.atan2(2.0 * self.L * math.sin(math.atan2(tgt[1], tgt[0])), ld)
            delta = max(-self.max_steer, min(self.max_steer, delta))
            self.steer += self.steer_filter * (delta - self.steer)
        return self.speed, self.steer
