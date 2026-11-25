"""신호등 미션 상태기계 (ROS 무관).

WAIT ─(빨강/노랑 확인)→ APPROACH(정지선까지 감속, 초록이면 취소) → HOLD(초록 기다림) ─(초록)→ PASS → DONE
정지선이 아직 안 보이면 approach_speed 로 다가간다. 정지선은 lane_node 의 횡단보도/정지선 거리.
"""

import math


class StopAtLine:
    """앞바퀴를 선 앞 stop_before 위치에 세우는 속도 프로파일."""

    def __init__(self, stop_before=0.05, decel=1.5, creep_speed=0.12, creep_dist=0.10,
                 tolerance=0.01, min_measure=0.03, max_measure=1.2, blend=0.4):
        self.stop_before = stop_before
        self.decel = decel
        self.creep_speed = creep_speed
        self.creep_dist = creep_dist
        self.tolerance = tolerance
        self.min_measure = min_measure
        self.max_measure = max_measure
        self.blend = blend
        self.target_s = None

    def reset(self):
        self.target_s = None

    def observe(self, line_dist, s_now):
        """line_dist: 앞바퀴→선 거리 (m, 없으면 <0)."""
        if line_dist is None or not (self.min_measure <= line_dist <= self.max_measure):
            return
        est = s_now + line_dist - self.stop_before
        if self.target_s is None:
            self.target_s = est
        else:
            self.target_s += self.blend * (est - self.target_s)

    def remaining(self, s_now):
        return None if self.target_s is None else self.target_s - s_now

    def speed(self, s_now, v_lane):
        rem = self.remaining(s_now)
        if rem is None:
            return v_lane
        if rem <= self.tolerance:
            return 0.0
        v = math.sqrt(2.0 * self.decel * rem)
        if rem < self.creep_dist:
            v = max(v, self.creep_speed)
        return min(v_lane, v)


class SignalMission:
    WAIT, APPROACH, HOLD, PASS, DONE = 'WAIT', 'APPROACH', 'HOLD', 'PASS', 'DONE'
    RED, YELLOW, GREEN = 0, 1, 2

    def __init__(self, stop: StopAtLine, approach_speed=0.35, signal_timeout=1.0,
                 green_confirm=2, pass_distance=1.0, stop_speed=0.02, max_count=1,
                 blind_stop_sec=4.0):
        self.stop = stop
        self.approach_speed = approach_speed
        self.signal_timeout = signal_timeout
        self.green_confirm = green_confirm
        self.pass_distance = pass_distance
        self.stop_speed = stop_speed
        self.max_count = max_count
        self.blind_stop_sec = blind_stop_sec
        self.state = self.WAIT
        self.count = 0
        self.signal = None
        self.signal_t = -1e9
        self.green_n = 0
        self.pass_until = None
        self.approach_t = None

    def on_signal(self, code, t):
        self.signal = code
        self.signal_t = t
        self.green_n = self.green_n + 1 if code == self.GREEN else 0

    def step(self, t, s, v_meas, line_dist, v_lane):
        fresh = t - self.signal_t < self.signal_timeout
        if self.state == self.WAIT:
            if (self.max_count <= 0 or self.count < self.max_count) and fresh \
                    and self.signal in (self.RED, self.YELLOW):
                self.state = self.APPROACH
                self.stop.reset()
                self.approach_t = t
            else:
                return None
        if self.state == self.APPROACH:
            if self.green_n >= self.green_confirm:
                self.state = self.WAIT  # 다가가는 중 초록 → 그냥 통과
                return None
            self.stop.observe(line_dist, s)
            if self.stop.target_s is None:
                # 정지선을 아직 못 봄: 천천히 접근, 너무 오래 못 보면 그 자리 정지
                if t - self.approach_t > self.blind_stop_sec:
                    v = 0.0
                else:
                    return min(v_lane, self.approach_speed)
            else:
                v = self.stop.speed(s, v_lane)
            if v == 0.0 and abs(v_meas) < self.stop_speed:
                self.state = self.HOLD
            return v
        if self.state == self.HOLD:
            if self.green_n >= self.green_confirm:
                self.state = self.PASS
                self.count += 1
                self.pass_until = s + self.pass_distance
                return None
            return 0.0
        if self.state == self.PASS:
            if s >= self.pass_until:
                self.state = self.DONE if 0 < self.max_count <= self.count else self.WAIT
            return None
        return None
