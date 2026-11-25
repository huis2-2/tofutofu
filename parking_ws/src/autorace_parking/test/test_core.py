import math

from autorace_parking.core import Parking


def test_sequence():
    p = Parking(slot_width=0.5, turn_radius=0.55, enter_distance=0.3)
    assert p.step(0.0, -1, 0.0, 1.0) is None
    p.on_sign(0.0)
    s = 0.0
    states = []
    grid_s = 2.0  # 뒷바퀴 기준 구역 시작 위치
    while p.state != 'PARKED' and s < 10:
        dist = grid_s - s - 0.26
        out = p.step(s, dist if 0.05 < dist < 1.2 else -1, 0.0, 1.0)
        states.append((p.state, s, out))
        s += 0.005
    turn = [x for x in states if x[0] == 'TURN']
    assert abs(turn[0][1] - (2.0 + 0.75 - 0.55)) < 0.02
    assert turn[0][2][1] < 0  # 오른쪽 최대 조향
    assert abs((turn[-1][1] - turn[0][1]) - 0.55 * math.pi / 2) < 0.02
    assert p.step(s, -1, 0.0, 1.0) == (0.0, 0.0)
