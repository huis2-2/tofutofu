"""횡단보도 정지 상태기계 (ROS 무관).

WAIT ─(횡단보도 거리 측정됨)→ APPROACH ─(목표 위치 도달 & 정지)→ HOLD ─(hold_sec)→ PASS ─(통과)→ DONE/WAIT

정지 위치: 엔코더 누적거리 s 로 '앞바퀴가 정지선(횡단보도 앞 가장자리 - stop_before)에 오는 s' 를 잡고
감속 곡선 v = sqrt(2 a d) 로 다가간다. 카메라가 가까이에서 선을 못 봐도 엔코더로 마무리한다.
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


class CrosswalkMission:
    WAIT, APPROACH, HOLD, PASS, DONE = 'WAIT', 'APPROACH', 'HOLD', 'PASS', 'DONE'

    def __init__(self, stop: StopAtLine, trigger_dist=1.0, hold_sec=3.5, stop_speed=0.02,
                 pass_distance=0.8, max_count=1, signal_ignore_sec=4.0):
        self.stop = stop
        self.trigger_dist = trigger_dist
        self.hold_sec = hold_sec
        self.stop_speed = stop_speed
        self.pass_distance = pass_distance
        self.max_count = max_count
        self.signal_ignore_sec = signal_ignore_sec
        self.state = self.WAIT
        self.count = 0
        self.stopped_at = None
        self.pass_until = None
        self.last_signal_t = -1e9

    def on_signal(self, t):
        self.last_signal_t = t

    def step(self, t, s, v_meas, crosswalk_dist, v_lane):
        """반환: 명령 속도 (None = 이 미션 비활성, lane 에 맡김)."""
        if self.state == self.WAIT:
            near_signal = t - self.last_signal_t < self.signal_ignore_sec
            if (self.count < self.max_count or self.max_count <= 0) and not near_signal \
                    and 0 <= crosswalk_dist <= self.trigger_dist:
                self.state = self.APPROACH
                self.stop.reset()
            else:
                return None
        if self.state == self.APPROACH:
            self.stop.observe(crosswalk_dist, s)
            v = self.stop.speed(s, v_lane)
            if v == 0.0 and abs(v_meas) < self.stop_speed:
                self.state = self.HOLD
                self.stopped_at = t  # 완전 정지 확인 시점부터 3초 카운트
            return v
        if self.state == self.HOLD:
            if abs(v_meas) >= self.stop_speed:
                self.stopped_at = t  # 아직 밀리는 중이면 다시 셈
            if t - self.stopped_at >= self.hold_sec:
                self.state = self.PASS
                self.count += 1
                self.pass_until = s + self.pass_distance
                return None
            return 0.0
        if self.state == self.PASS:
            if s >= self.pass_until:
                self.state = self.DONE if 0 < self.max_count <= self.count else self.WAIT
            return None
        return None  # DONE
