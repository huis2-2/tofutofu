"""Pure Pursuit + 곡률 기반 속도. ROS 와 무관."""

from dataclasses import dataclass
import math

import numpy as np


@dataclass
class ControlParams:
    wheelbase: float = 0.26
    lookahead_min: float = 0.35
    lookahead_max: float = 1.20
    lookahead_base: float = 0.25   # Ld = base + gain * v
    lookahead_gain: float = 0.45
    max_steer: float = 0.349       # rad (좌 20도 / 우 23도 중 작은 값)
    steer_filter: float = 0.6      # 0~1, 1 = 필터 없음
    v_max: float = 1.2             # m/s 직선 최고 속도
    v_min: float = 0.35            # 커브 최저 속도
    v_boost: float = 1.6           # no_speed_limit 구간
    a_lat_max: float = 1.6         # 횡가속도 한계 → 커브 속도
    accel: float = 1.5             # 가속 제한 m/s^2 (감속은 즉시)
    heading_speed_gain: float = 0.6  # 헤딩 오차가 크면 감속
    lost_hold_sec: float = 0.4     # 차선 놓치면 이 시간 동안 직전 조향 유지
    lost_speed: float = 0.3


def lookahead_point(path, ld):
    """path: (N,2), 원점에서 거리 ld 인 경로 위 점 (선형 보간, 짧으면 연장)."""
    d = np.hypot(path[:, 0], path[:, 1])
    idx = np.nonzero(d >= ld)[0]
    if len(idx) == 0:
        # 경로 끝을 마지막 방향으로 연장
        if len(path) >= 2:
            p1, p0 = path[-1], path[-2]
            v = p1 - p0
            n = np.linalg.norm(v)
            if n > 1e-6:
                v = v / n
                # |p1 + t v| = ld 를 푸는 t
                b = 2 * p1.dot(v)
                c = p1.dot(p1) - ld * ld
                t = (-b + math.sqrt(max(0.0, b * b - 4 * c))) / 2
                return p1 + t * v
        return path[-1]
    i = idx[0]
    if i == 0:
        return path[0]
    p0, p1 = path[i - 1], path[i]
    d0, d1 = d[i - 1], d[i]
    t = (ld - d0) / max(1e-6, d1 - d0)
    return p0 + t * (p1 - p0)


def pure_pursuit(path, speed, p: ControlParams):
    ld = float(np.clip(p.lookahead_base + p.lookahead_gain * max(0.0, speed),
                       p.lookahead_min, p.lookahead_max))
    tx, ty = lookahead_point(path, ld)
    ld_act = max(1e-3, math.hypot(tx, ty))
    alpha = math.atan2(ty, tx)
    delta = math.atan2(2.0 * p.wheelbase * math.sin(alpha), ld_act)
    return float(np.clip(delta, -p.max_steer, p.max_steer)), (tx, ty), ld


def curve_speed(path, heading, p: ControlParams, v_cap):
    """경로 전체의 최대 곡률로 횡가속도 한계 속도."""
    if len(path) >= 3:
        poly = np.polyfit(path[:, 0], path[:, 1], 2)
        xs = path[:, 0]
        d1 = np.polyval(np.polyder(poly), xs)
        d2 = np.polyval(np.polyder(poly, 2), xs)
        kappa = float(np.max(np.abs(d2) / (1 + d1 * d1) ** 1.5))
    else:
        kappa = 0.0
    v = v_cap if kappa < 1e-3 else min(v_cap, math.sqrt(p.a_lat_max / kappa))
    v *= max(0.3, 1.0 - p.heading_speed_gain * abs(heading))
    return float(np.clip(v, p.v_min, v_cap)), kappa


class LaneController:

    def __init__(self, params=None):
        self.p = params or ControlParams()
        self.steer = 0.0
        self.speed_cmd = 0.0
        self.last_valid_t = None

    def step(self, t, dt, valid, path, heading, measured_speed, boost=False):
        """반환: (speed m/s, steering rad, info dict)."""
        p = self.p
        info = {}
        if valid and path is not None and len(path) >= 2:
            self.last_valid_t = t
            delta, target, ld = pure_pursuit(path, measured_speed, p)
            self.steer += p.steer_filter * (delta - self.steer)
            v_cap = p.v_boost if boost else p.v_max
            v, kappa = curve_speed(path, heading, p, v_cap)
            info = {'target': target, 'ld': ld, 'kappa': kappa}
        elif self.last_valid_t is not None and t - self.last_valid_t < p.lost_hold_sec:
            v = min(self.speed_cmd, p.lost_speed)
        else:
            v = 0.0
        # 가속만 제한 (감속은 즉시)
        if v > self.speed_cmd:
            v = min(v, self.speed_cmd + p.accel * dt)
        self.speed_cmd = v
        return v, self.steer, info
