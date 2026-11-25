from autorace_signal.core import SignalMission, StopAtLine


def test_red_then_green():
    m = SignalMission(StopAtLine(stop_before=0.05))
    s, v, t, dt = 0.0, 0.0, 0.0, 0.02
    line_s = 2.0
    hold_t = None
    for i in range(1500):
        if t < 8.0:
            m.on_signal(SignalMission.RED, t)
        else:
            m.on_signal(SignalMission.GREEN, t)
        dist = line_s - s
        cmd = m.step(t, s, v, dist if 0 < dist < 1.2 else -1.0, 1.0)
        v = 1.0 if cmd is None else cmd
        if m.state == 'HOLD' and hold_t is None:
            hold_t = t
            assert abs(s - 1.95) < 0.03, s
        s += v * dt
        t += dt
    assert hold_t is not None and m.state in ('PASS', 'DONE') and s > 2.5


def test_green_during_approach_passes():
    m = SignalMission(StopAtLine())
    m.on_signal(SignalMission.RED, 0.0)
    assert m.step(0.0, 0.0, 1.0, 0.8, 1.0) is not None
    m.on_signal(SignalMission.GREEN, 0.1)
    m.on_signal(SignalMission.GREEN, 0.15)
    assert m.step(0.2, 0.1, 1.0, 0.7, 1.0) is None
