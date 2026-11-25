from autorace_crosswalk.core import CrosswalkMission, StopAtLine


def simulate(mission, line_s, v_lane=1.0, dt=0.02, steps=2000, signal=False):
    """1차원 차량: 명령 속도로 즉시 반응 (브레이크 이상적). 앞바퀴→선 거리 = line_s - s."""
    s, v, t = 0.0, 0.0, 0.0
    log = []
    for _ in range(steps):
        dist = line_s - s
        cmd = mission.step(t, s, v, dist if 0 < dist < 1.2 else -1.0, v_lane)
        v = v_lane if cmd is None else cmd
        s += v * dt
        t += dt
        log.append((t, s, v, mission.state))
    return log


def test_stops_at_line_and_waits():
    m = CrosswalkMission(StopAtLine(stop_before=0.05), hold_sec=3.5)
    log = simulate(m, line_s=3.0)
    stopped = [(t, s) for t, s, v, st in log if st == 'HOLD']
    assert stopped, 'HOLD 상태가 없음'
    s_stop = stopped[0][1]
    assert abs(s_stop - (3.0 - 0.05)) < 0.03, s_stop
    hold_time = stopped[-1][0] - stopped[0][0]
    assert hold_time >= 3.4
    assert log[-1][3] in ('DONE', 'PASS') and log[-1][2] > 0  # 다시 출발


def test_ignored_near_signal():
    m = CrosswalkMission(StopAtLine())
    m.on_signal(0.0)
    assert m.step(0.1, 0.0, 1.0, 0.5, 1.0) is None
