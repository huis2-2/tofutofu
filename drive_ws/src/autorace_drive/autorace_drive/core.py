"""조향각 → 서보 µs, 엔코더 속도 PID → ESC µs. ROS / 하드웨어 무관."""

from dataclasses import dataclass


@dataclass
class SteerParams:
    center_us: float = 1660.0
    left_us: float = 2000.0      # 최대 왼쪽 펄스
    right_us: float = 1400.0     # 최대 오른쪽 펄스
    max_left_rad: float = 0.45   # left_us 일 때 앞바퀴 각도 (실측)
    max_right_rad: float = 0.45
    trim_us: float = 0.0


def steer_to_us(angle, p: SteerParams):
    """angle rad (+ 왼쪽) → µs (선형, 좌우 따로)."""
    if angle >= 0:
        r = min(1.0, angle / p.max_left_rad)
        us = p.center_us + r * (p.left_us - p.center_us)
    else:
        r = min(1.0, -angle / p.max_right_rad)
        us = p.center_us - r * (p.center_us - p.right_us)
    return us + p.trim_us


def steer_ratio(us, p: SteerParams):
    """drive_record.py 와 같은 -1(오른쪽) ~ +1(왼쪽) 비율."""
    if us >= p.center_us:
        return (us - p.center_us) / (p.left_us - p.center_us)
    return -(p.center_us - us) / (p.center_us - p.right_us)


@dataclass
class SpeedParams:
    neutral_us: float = 1500.0
    deadband_us: float = 40.0    # 이만큼은 넣어야 바퀴가 돌기 시작 (실측)
    kff: float = 120.0           # µs per m/s (피드포워드, 실측)
    kp: float = 80.0
    ki: float = 120.0
    kd: float = 0.0
    i_limit: float = 120.0       # 적분 항 최대 µs
    max_forward_us: float = 1750.0
    brake_us: float = 1350.0     # 정지 명령 + 아직 움직이면 브레이크 (ESC 역방향 = 브레이크)
    brake_speed: float = 0.10    # 이 속도 이상일 때만 브레이크
    accel: float = 3.0           # 목표 속도 증가율 제한 m/s^2
    stop_speed: float = 0.02     # 목표가 이보다 작으면 정지
    open_loop: bool = False      # 엔코더 없을 때 피드포워드만


class SpeedPID:

    def __init__(self, p: SpeedParams):
        self.p = p
        self.i = 0.0
        self.prev_err = 0.0
        self.target = 0.0

    def reset(self):
        self.i = 0.0
        self.prev_err = 0.0

    def step(self, target, measured, dt, encoder_ok=True):
        p = self.p
        if target > self.target:
            target = min(target, self.target + p.accel * dt)
        self.target = target
        if target < p.stop_speed:
            self.reset()
            if encoder_ok and measured > p.brake_speed:
                return p.brake_us
            return p.neutral_us
        ff = p.neutral_us + p.deadband_us + p.kff * target
        if p.open_loop or not encoder_ok:
            self.reset()
            return min(p.max_forward_us, ff)
        err = target - measured
        d = (err - self.prev_err) / dt if dt > 0 else 0.0
        self.prev_err = err
        self.i = max(-p.i_limit, min(p.i_limit, self.i + p.ki * err * dt))
        us = ff + p.kp * err + self.i + p.kd * d
        # 전진 중에는 중립 아래로 내리지 않음 (감속은 중립 + 마찰로)
        return max(p.neutral_us, min(p.max_forward_us, us))
