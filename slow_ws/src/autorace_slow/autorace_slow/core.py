"""서행 상태기계 (ROS 무관).

트리거 (하나라도): 서행 표지(slow), 노면 마름모(diamond), 지그재그 서행 유도선(zigzag)
→ 마지막 트리거 위치 + hold_distance 까지 서행. 지그재그가 보이는 동안은 계속 서행.
속도 = min(lane 속도 * ratio, max_speed). 대회 기준: 육안으로 느려진 게 보이면 됨.
"""


class SlowMission:

    def __init__(self, ratio=0.45, max_speed=0.45, min_speed=0.15, hold_distance=1.2,
                 zigzag_hold_distance=0.4, diamond_trigger_dist=0.8):
        self.ratio = ratio
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.hold_distance = hold_distance
        self.zigzag_hold_distance = zigzag_hold_distance
        self.diamond_trigger_dist = diamond_trigger_dist
        self.until = -1e9
        self.reason = ''

    def trigger(self, s, distance, reason):
        if s + distance > self.until:
            self.until = s + distance
            self.reason = reason

    def step(self, s, v_lane, slow_sign=False, diamond_dist=-1.0, zigzag=0):
        if slow_sign:
            self.trigger(s, self.hold_distance, 'sign')
        if 0 <= diamond_dist <= self.diamond_trigger_dist:
            self.trigger(s, self.hold_distance + diamond_dist, 'diamond')
        if zigzag > 0:
            self.trigger(s, self.zigzag_hold_distance, 'zigzag')
        if s > self.until:
            return None
        return max(self.min_speed, min(v_lane * self.ratio, self.max_speed)) if v_lane > 0 else 0.0
