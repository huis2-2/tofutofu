"""T자 주차 (전면 주차, 오른쪽 주차 구역의 윗줄 가운데 칸). ROS 무관.

트랙: 주차 구역(2줄 x 3칸)은 마지막 직선 구간 오른쪽에 붙어 있고, 윗줄이 차로 쪽.
차로를 따라가며 오른쪽에 처음 보이는 주차선 = 구역 시작 가장자리 (grid_start).
가운데 칸 중심 = grid_start + 1.5 * slot_width.

동작 (엔코더 거리 s 기준, 뒷바퀴 원점):
  ARMED   : P 표지 확인 후 approach_speed 로 lane 추종, 주차선 거리로 grid_start 추정
  APPROACH: 뒷바퀴가 turn_s = 칸중심 - turn_radius + turn_offset 에 오면
  TURN    : 오른쪽 최대 조향으로 원호 (turn_radius * pi/2 * turn_scale) 만큼 전진 → 90도 회전
  ENTER   : 직진 enter_distance (칸 안으로)
  PARKED  : 정지 유지 (대회 끝)
turn_radius 는 최대 조향으로 원을 그려 실측 (지름/2). turn_offset 으로 미세 조정.
"""

import math


class Parking:
    IDLE, ARMED, APPROACH, TURN, ENTER, PARKED = 'IDLE', 'ARMED', 'APPROACH', 'TURN', 'ENTER', 'PARKED'

    def __init__(self, wheelbase=0.26, slot_width=0.50, slot_index=1.5, turn_radius=0.613,
                 turn_offset=0.0, turn_scale=1.0, enter_distance=0.35, approach_speed=0.3,
                 maneuver_speed=0.25, max_steer=0.401, side=-1, min_measure=0.05, max_measure=1.2,
                 arm_timeout_dist=6.0):
        self.L = wheelbase
        self.slot_width = slot_width
        self.slot_index = slot_index
        self.R = turn_radius
        self.turn_offset = turn_offset
        self.turn_scale = turn_scale
        self.enter_distance = enter_distance
        self.approach_speed = approach_speed
        self.maneuver_speed = maneuver_speed
        self.max_steer = max_steer
        self.side = side  # -1 오른쪽 주차
        self.min_measure = min_measure
        self.max_measure = max_measure
        self.arm_timeout_dist = arm_timeout_dist
        self.state = self.IDLE
        self.grid_s = None   # 구역 시작 가장자리의 s (뒷바퀴 기준)
        self.mark_s = None
        self.arm_s = None

    def on_sign(self, s):
        if self.state == self.IDLE:
            self.state = self.ARMED
            self.arm_s = s

    def observe(self, s, parking_dist):
        """parking_dist: 앞바퀴→주차선(오른쪽) 가장 가까운 거리."""
        if parking_dist is None or not (self.min_measure <= parking_dist <= self.max_measure):
            return
        est = s + parking_dist + self.L  # 뒷바퀴 기준 위치로
        if self.grid_s is None:
            self.grid_s = est
        else:
            # 가장 가까운 선이 구역 시작이므로, 더 앞쪽 값(다음 칸 선)으로 끌려가지 않게 작은 쪽 우선
            self.grid_s = min(self.grid_s, est) if est < self.grid_s + 0.1 else self.grid_s

    def turn_s(self):
        return self.grid_s + self.slot_index * self.slot_width - self.R + self.turn_offset

    def step(self, s, parking_dist, lane_steer, v_lane):
        """반환: (speed, steer) 또는 None (비활성 → lane)."""
        if self.state == self.IDLE:
            return None
        if self.state == self.ARMED:
            self.observe(s, parking_dist)
            if self.grid_s is not None:
                self.state = self.APPROACH
            elif s - self.arm_s > self.arm_timeout_dist:
                self.state = self.IDLE  # 오검출
                return None
            else:
                return min(v_lane, self.approach_speed), lane_steer
        if self.state == self.APPROACH:
            if s < self.grid_s - 0.3:
                self.observe(s, parking_dist)
            if s >= self.turn_s():
                self.state = self.TURN
                self.mark_s = s
            else:
                return min(v_lane, self.approach_speed), lane_steer
        if self.state == self.TURN:
            arc = self.R * math.pi / 2.0 * self.turn_scale
            if s - self.mark_s >= arc:
                self.state = self.ENTER
                self.mark_s = s
            else:
                return self.maneuver_speed, self.side * self.max_steer
        if self.state == self.ENTER:
            if s - self.mark_s >= self.enter_distance:
                self.state = self.PARKED
            else:
                return self.maneuver_speed, 0.0
        return 0.0, 0.0  # PARKED: 계속 정지
