"""정적 장애물(LIMO 7대) 회피: LiDAR 로 앞 차로가 막혔는지 보고 비어 있는 옆 차로로 변경.

차선 변경 자체는 lane_node 에 /lane/change (+1 왼쪽 / -1 오른쪽) 를 보내서
추적 차선을 옆으로 옮기는 방식 → 조향은 lane Pure Pursuit 가 그대로 담당.

규칙
- 활성: YOLO fixed_obstacle 확인 또는 LiDAR 로 앞 차로 막힘 (require_yolo 이면 YOLO 필수)
- 앞 차로(|y| < corridor_half, front < x < check_dist)에 점이 min_points 이상 → 막힘
- 왼쪽/오른쪽 차로(y = ±lane_width 근처)가 비어 있고, 그쪽 경계가 점선이면 그쪽으로 변경
- 변경 후 change_cooldown 거리 동안 추가 변경 금지 (차가 옆 차로로 넘어가는 시간)
- 주변 장애물이 exit_distance 동안 안 보이면 종료
"""


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


def count_box(pts, x0, x1, y0, y1):
    if len(pts) == 0:
        return 0
    m = (pts[:, 0] > x0) & (pts[:, 0] < x1) & (pts[:, 1] > y0) & (pts[:, 1] < y1)
    return int(m.sum())


class ObjAvoid:

    def __init__(self, lane_width=0.55, front=0.26, check_dist=0.9, side_extra=0.35,
                 corridor_half=0.17, min_points=4, change_cooldown=0.6, exit_distance=0.8,
                 avoid_speed=0.5, require_dashed=True, prefer=1, active_timeout_dist=2.5,
                 require_yolo=True):
        self.w = lane_width
        self.front = front
        self.check_dist = check_dist
        self.side_extra = side_extra
        self.corridor_half = corridor_half
        self.min_points = min_points
        self.change_cooldown = change_cooldown
        self.exit_distance = exit_distance
        self.avoid_speed = avoid_speed
        self.require_dashed = require_dashed
        self.prefer = 1 if prefer >= 0 else -1
        self.active_timeout_dist = active_timeout_dist
        self.require_yolo = require_yolo
        self.active = False
        self.armed_until = -1e9      # YOLO 로 무장된 거리 범위
        self.last_change_s = -1e9
        self.last_seen_s = -1e9
        self.changes = 0

    def on_yolo(self, s):
        self.armed_until = s + self.active_timeout_dist

    def blocked(self, pts):
        return count_box(pts, self.front, self.front + self.check_dist,
                         -self.corridor_half, self.corridor_half) >= self.min_points

    def side_free(self, pts, side):
        yc = side * self.w
        return count_box(pts, self.front - 0.25, self.front + self.check_dist + self.side_extra,
                         yc - self.corridor_half - 0.05, yc + self.corridor_half + 0.05) < self.min_points

    def step(self, s, pts, left_dashed, right_dashed, v_lane):
        """반환: (speed or None, lane_change -1/0/+1)."""
        armed = s < self.armed_until or not self.require_yolo
        near = count_box(pts, 0.0, self.front + 1.3, -self.w * 1.6, self.w * 1.6) >= self.min_points
        if near:
            self.last_seen_s = s
        if not self.active:
            if armed and self.blocked(pts):
                self.active = True
                self.last_seen_s = s
            else:
                return None, 0
        change = 0
        if self.blocked(pts) and s - self.last_change_s > self.change_cooldown:
            ok = {1: left_dashed or not self.require_dashed,
                  -1: right_dashed or not self.require_dashed}
            for side in (self.prefer, -self.prefer):
                if ok[side] and self.side_free(pts, side):
                    change = side
                    break
            if change:
                self.last_change_s = s
                self.changes += 1
        if s - self.last_seen_s > self.exit_distance and s - self.last_change_s > self.change_cooldown:
            self.active = False
            return None, 0
        # 막혔는데 갈 곳이 없으면 정지 (충돌 -1점 방지), 아니면 회피 속도
        if self.blocked(pts) and change == 0 and s - self.last_change_s > self.change_cooldown:
            nearest = pts[(np.abs(pts[:, 1]) < self.corridor_half) & (pts[:, 0] > self.front)]
            gap = float(nearest[:, 0].min() - self.front) if len(nearest) else 1.0
            return (0.0 if gap < 0.25 else min(v_lane, self.avoid_speed * 0.5)), 0
        return min(v_lane, self.avoid_speed), change
