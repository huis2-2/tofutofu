"""지면 좌표의 차선 점들 → 좌/우 경계 → 중앙 경로. ROS 와 무관 (테스트 가능).

좌표: x 앞, y 왼쪽, 원점 = 뒷바퀴 축 (m)

알고리즘
1) 경계선 점 = yellow_line + dashed_line + zigzag_line
2) 좌/우 경계를 앞쪽으로 bin 단위 추적 (sliding window). 시작 예상 위치는
   직전 프레임 다항식 (없으면 중심 ± 차선폭/2)
3) 각 경계를 y = a x^2 + b x + c 로 피팅. 지그재그 쪽은 1차(직선)로 피팅 → 직진 헤딩 유지
4) 중앙 경로: 양쪽 다 있으면 평균, 한쪽만 있으면 경계에서 법선 방향으로 차선폭/2
5) 갈림길 bias: +1(A) 왼쪽 경계만, -1(B) 오른쪽 경계만 사용
6) 차선 변경 요청(+1 왼쪽 / -1 오른쪽): 추적 상태를 차선폭만큼 옮겨 옆 차선을 잡는다
7) 특징 거리: 정지선/횡단보도/주차선/마름모의 앞바퀴 기준 거리
"""

from dataclasses import dataclass, field
import math

import numpy as np

YELLOW, DASHED, HORIZONTAL, ZIGZAG, CROSSWALK, PARKING, DIAMOND = 1, 2, 3, 4, 5, 6, 7
BOUNDARY_IDS = (YELLOW, DASHED, ZIGZAG)

# /lane/target (Float32MultiArray) 인덱스
T_VALID, T_LAT, T_HEADING, T_CURV, T_STOP, T_CROSS, T_PARK, T_DIAMOND, \
    T_LEFT, T_RIGHT, T_LEFT_DASHED, T_RIGHT_DASHED, T_ZIGZAG, T_BIAS, T_WIDTH = range(15)
TARGET_LEN = 15


@dataclass
class LaneParams:
    lane_width: float = 0.55
    x_min: float = 0.30          # 카메라에 보이는 가장 가까운 지면 (뒷바퀴 기준)
    x_max: float = 1.20
    bin_size: float = 0.08
    margin: float = 0.12         # 예상 위치 ± margin 안의 점만 사용
    first_margin: float = 0.20   # 이전 추정이 없을 때 첫 bin 탐색 폭
    min_points_bin: int = 3
    min_bins: int = 3
    max_gap_bins: int = 5        # 점선 틈 허용
    start_window: float = 0.35   # 이전 추정이 없으면 차 가까이(x_min+이 값 안)에서 시작해야 함
    wheelbase: float = 0.26      # 원점(뒷바퀴) → 앞바퀴
    x_ref: float = 0.55          # 횡오차/헤딩을 재는 앞쪽 거리
    feature_min_points: int = 15
    zigzag_min_points: int = 30
    width_min_ratio: float = 0.6  # 좌우 간격이 이보다 좁으면 같은 선을 잡은 것
    width_max_ratio: float = 1.6
    path_points: int = 16


@dataclass
class Side:
    poly: np.ndarray = None      # y(x) 계수 (np.polyfit 순서)
    n_bins: int = 0
    x_far: float = 0.0           # 실제로 점을 찾은 가장 먼 x
    dashed_ratio: float = 0.0
    zigzag: bool = False


@dataclass
class LaneResult:
    valid: bool = False
    path: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    left: Side = field(default_factory=Side)
    right: Side = field(default_factory=Side)
    lateral_error: float = 0.0
    heading: float = 0.0
    curvature: float = 0.0
    stop_line: float = -1.0
    crosswalk: float = -1.0
    parking: float = -1.0
    diamond: float = -1.0
    zigzag: int = 0
    width: float = 0.55

    def target_array(self, bias):
        t = [0.0] * TARGET_LEN
        t[T_VALID] = 1.0 if self.valid else 0.0
        t[T_LAT] = self.lateral_error
        t[T_HEADING] = self.heading
        t[T_CURV] = self.curvature
        t[T_STOP] = self.stop_line
        t[T_CROSS] = self.crosswalk
        t[T_PARK] = self.parking
        t[T_DIAMOND] = self.diamond
        t[T_LEFT] = 1.0 if self.left.poly is not None else 0.0
        t[T_RIGHT] = 1.0 if self.right.poly is not None else 0.0
        t[T_LEFT_DASHED] = self.left.dashed_ratio
        t[T_RIGHT_DASHED] = self.right.dashed_ratio
        t[T_ZIGZAG] = float(self.zigzag)
        t[T_BIAS] = float(bias)
        t[T_WIDTH] = self.width
        return t


def _polyval(poly, x):
    return np.polyval(poly, x)


def _shift_poly(poly, dy):
    if poly is None:
        return None
    p = np.array(poly, np.float64)
    p[-1] += dy
    return p


class LaneEstimator:

    def __init__(self, params=None):
        self.p = params or LaneParams()
        self.prev_left = None
        self.prev_right = None
        self.width = self.p.lane_width
        self.lost_frames = 0

    # ----- 상태 조작 -----
    def lane_change(self, direction):
        """direction +1: 왼쪽 차선으로, -1: 오른쪽 차선으로."""
        w = self.width
        if direction > 0:
            base = self.prev_left if self.prev_left is not None else (
                _shift_poly(self.prev_right, w))
            self.prev_right = base
            self.prev_left = _shift_poly(base, w)
        elif direction < 0:
            base = self.prev_right if self.prev_right is not None else (
                _shift_poly(self.prev_left, -w))
            self.prev_left = base
            self.prev_right = _shift_poly(base, -w)

    def reset(self):
        self.prev_left = None
        self.prev_right = None

    # ----- 추적 -----
    def _track(self, pts, cls, prev, y0, other_poly=None):
        p = self.p
        side = Side()
        if len(pts) == 0:
            return side
        edges = np.arange(p.x_min, p.x_max + 1e-6, p.bin_size)
        xs, ys, n_dashed, n_total, n_zig = [], [], 0, 0, 0
        gap = 0
        order = np.argsort(pts[:, 0])
        sx, sy, sc = pts[order, 0], pts[order, 1], cls[order]
        idx = np.searchsorted(sx, edges)
        for i in range(len(edges) - 1):
            xc = 0.5 * (edges[i] + edges[i + 1])
            if len(xs) >= 2:
                # 이번 프레임에서 찾은 점으로 선형 외삽 (+ 이전 다항식과 평균)
                slope = (ys[-1] - ys[-2]) / max(1e-6, xs[-1] - xs[-2])
                y_exp = ys[-1] + slope * (xc - xs[-1])
                if prev is not None:
                    y_exp = 0.5 * (y_exp + _polyval(prev, xc))
                margin = p.margin
            elif len(xs) == 1:
                y_exp = ys[-1] if prev is None else 0.5 * (ys[-1] + _polyval(prev, xc))
                margin = p.margin
            else:
                y_exp = _polyval(prev, xc) if prev is not None else y0
                margin = p.margin if prev is not None else p.first_margin
            if not xs and prev is None and xc > p.x_min + p.start_window:
                break  # 먼 곳에서 처음 잡히는 선은 반대편 선이 휘어 들어온 것일 수 있다
            a, b = idx[i], idx[i + 1]
            if b - a < p.min_points_bin:
                gap += 1
                if gap > p.max_gap_bins and xs:
                    break
                continue
            by, bc = sy[a:b], sc[a:b]
            sel = np.abs(by - y_exp) < margin
            if other_poly is not None:
                # 반대쪽 경계와 너무 가까운 점은 제외 (같은 선 두 번 잡기 방지)
                sel &= np.abs(by - _polyval(other_poly, xc)) > p.lane_width * 0.35
            if sel.sum() < p.min_points_bin:
                gap += 1
                if gap > p.max_gap_bins and xs:
                    break
                continue
            gap = 0
            xs.append(xc)
            ys.append(float(np.median(by[sel])))
            n_total += int(sel.sum())
            n_dashed += int((bc[sel] == DASHED).sum())
            n_zig += int((bc[sel] == ZIGZAG).sum())
        if len(xs) < p.min_bins:
            return side
        side.zigzag = n_zig >= p.zigzag_min_points and n_zig > 0.3 * n_total
        if side.zigzag:
            # 지그재그: 모든 점 직선 피팅 → 좌우 흔들림 평균 = 직진 헤딩
            deg = 1
        else:
            deg = 2 if len(xs) >= 5 else 1
        poly = np.polyfit(xs, ys, deg)
        if deg == 1:
            poly = np.concatenate([[0.0], poly])
        side.poly = poly
        side.n_bins = len(xs)
        side.x_far = xs[-1]
        side.dashed_ratio = n_dashed / max(1, n_total)
        return side

    # ----- 메인 -----
    def update(self, xy, cls, bias=0):
        """xy: (N, 2) 지면 좌표, cls: (N,) 마스크 클래스 값. bias: +1 A(왼쪽) / -1 B(오른쪽) / 0."""
        p = self.p
        res = LaneResult(width=self.width)
        bmask = np.isin(cls, BOUNDARY_IDS)
        bpts, bcls = xy[bmask], cls[bmask]

        half = self.width / 2.0
        left = self._track(bpts, bcls, self.prev_left, half)
        right = self._track(bpts, bcls, self.prev_right, -half,
                            other_poly=left.poly)

        # 같은 선을 양쪽에서 잡았거나 너무 넓으면 신뢰 낮은 쪽 버림
        if left.poly is not None and right.poly is not None:
            xm = np.linspace(p.x_min, min(p.x_max, p.x_min + 0.6), 5)
            gap = float(np.mean(_polyval(left.poly, xm) - _polyval(right.poly, xm)))
            if gap < p.width_min_ratio * p.lane_width or gap > p.width_max_ratio * p.lane_width:
                if left.n_bins >= right.n_bins:
                    right = Side()
                else:
                    left = Side()
            else:
                self.width = 0.9 * self.width + 0.1 * float(np.clip(
                    gap, 0.8 * p.lane_width, 1.2 * p.lane_width))

        # 갈림길: 지시 방향 경계만 사용
        if bias > 0 and left.poly is not None:
            right = Side()
        elif bias < 0 and right.poly is not None:
            left = Side()

        res.left, res.right = left, right
        res.zigzag = int(left.zigzag) + int(right.zigzag)
        res.width = self.width

        # 경로는 실제로 선이 보인 범위까지만 (보이지 않는 먼 곳으로 다항식 외삽 금지)
        x_far = max([s.x_far for s in (left, right) if s.poly is not None], default=p.x_max)
        x_end = float(np.clip(x_far + p.bin_size, p.x_min + 0.3, p.x_max))
        xs = np.linspace(p.x_min, x_end, p.path_points)
        half = self.width / 2.0
        if left.poly is not None and right.poly is not None:
            cy = 0.5 * (_polyval(left.poly, xs) + _polyval(right.poly, xs))
            path = np.column_stack([xs, cy])
        elif left.poly is not None or right.poly is not None:
            poly, sgn = (left.poly, -1.0) if left.poly is not None else (right.poly, 1.0)
            y = _polyval(poly, xs)
            dy = _polyval(np.polyder(poly), xs)
            norm = np.sqrt(1.0 + dy * dy)
            # 왼쪽 경계 → 오른쪽(법선 -)으로, 오른쪽 경계 → 왼쪽으로 half 만큼
            path = np.column_stack([xs - sgn * dy * half / norm, y + sgn * half / norm])
        else:
            path = None

        if path is None:
            self.lost_frames += 1
            if self.lost_frames > 15:
                self.reset()
            res.valid = False
        else:
            self.lost_frames = 0
            # 추적 상태 갱신 (없는 쪽은 반대편에서 추정)
            self.prev_left = left.poly if left.poly is not None else _shift_poly(right.poly, self.width)
            self.prev_right = right.poly if right.poly is not None else _shift_poly(left.poly, -self.width)
            res.valid = True
            res.path = path
            cpoly = np.polyfit(path[:, 0], path[:, 1], 2)
            xr = p.x_ref
            d1 = np.polyval(np.polyder(cpoly), xr)
            d2 = np.polyval(np.polyder(cpoly, 2), xr)
            res.lateral_error = float(np.polyval(cpoly, xr))
            res.heading = float(math.atan(d1))
            res.curvature = float(d2 / (1.0 + d1 * d1) ** 1.5)

        self._features(xy, cls, res)
        return res

    def _features(self, xy, cls, res):
        """앞바퀴 → 특징 가장 가까운 가장자리 거리 (m). 없으면 -1."""
        p = self.p
        if res.valid:
            cpoly = np.polyfit(res.path[:, 0], res.path[:, 1], 2)
        else:
            cpoly = np.array([0.0, 0.0, 0.0])
        half = self.width / 2.0

        def dist(cid, lo, hi):
            sel = cls == cid
            if sel.sum() < p.feature_min_points:
                return -1.0
            pts = xy[sel]
            dy = pts[:, 1] - np.polyval(cpoly, pts[:, 0])
            pts = pts[(dy > lo) & (dy < hi)]
            if len(pts) < p.feature_min_points:
                return -1.0
            return float(np.percentile(pts[:, 0], 5) - p.wheelbase)

        res.stop_line = dist(HORIZONTAL, -half, half)
        res.crosswalk = dist(CROSSWALK, -half, half)
        res.diamond = dist(DIAMOND, -half, half)
        res.parking = dist(PARKING, -half - 1.2, -half + 0.15)  # 오른쪽 주차 구역
