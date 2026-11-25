"""고정 우선순위 선택 (ROS 무관). 위에 있을수록 우선. lane 은 마지막 기본값."""

INPUTS = (
    ('crosswalk', '/mission/crosswalk/command_01'),
    ('signal', '/mission/signal/command_02'),
    ('parking', '/mission/parking/command_03'),
    ('slow', '/mission/slow/command_04'),
    ('objavoid', '/mission/objavoid/command_05'),
    ('cone', '/mission/cone/command_06'),
    ('lane', '/mission/lane'),
)


class Arbiter:

    def __init__(self, timeout=0.25, max_speed=2.0, max_steer=0.6):
        self.timeout = timeout
        self.max_speed = max_speed
        self.max_steer = max_steer
        self.latest = {}

    def receive(self, source, speed, steer, t):
        if not (speed == speed and steer == steer):  # NaN
            self.latest.pop(source, None)
            return
        self.latest[source] = (speed, steer, t)

    def select(self, t, disabled=()):
        """반환: (source, speed, steer). 유효 입력이 없으면 ('none', 0, 0)."""
        for source, _ in INPUTS:
            if source in disabled:
                continue
            c = self.latest.get(source)
            if c is None:
                continue
            if t - c[2] > self.timeout:
                del self.latest[source]
                continue
            speed = max(0.0, min(self.max_speed, c[0]))   # 후진 없음
            steer = max(-self.max_steer, min(self.max_steer, c[1]))
            return source, speed, steer
        return 'none', 0.0, 0.0
