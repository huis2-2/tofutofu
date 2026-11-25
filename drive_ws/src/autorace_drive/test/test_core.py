from autorace_drive.core import SpeedParams, SpeedPID, SteerParams, steer_ratio, steer_to_us


def test_steer_mapping():
    p = SteerParams()
    assert steer_to_us(0.0, p) == 1660
    assert steer_to_us(0.349, p) == 2000
    assert steer_to_us(-0.401, p) == 1400
    assert steer_to_us(-1.0, p) == 1400
    assert abs(steer_ratio(1830, p) - 0.5) < 1e-6


def test_pid_converges_and_brakes():
    p = SpeedParams()
    pid = SpeedPID(p)
    v = 0.0
    for _ in range(300):  # 단순 차량 모델: 정상상태 v = (us - 1765)/150
        us = pid.step(1.0, v, 0.02)
        v += (max(0.0, us - 1765) / 150.0 - v) * 0.1
    assert abs(v - 1.0) < 0.05
    assert pid.step(0.0, 0.8, 0.02) == p.brake_us
    assert pid.step(0.0, 0.0, 0.02) == p.neutral_us
