from autorace_slow.core import SlowMission


def test_sign_slows_for_distance():
    m = SlowMission(ratio=0.5, max_speed=1.0, hold_distance=1.0)
    assert m.step(0.0, 1.0) is None
    assert m.step(0.0, 1.0, slow_sign=True) == 0.5
    assert m.step(0.9, 1.0) == 0.5
    assert m.step(1.1, 1.0) is None


def test_zigzag_keeps_slow():
    m = SlowMission(ratio=0.5, max_speed=0.3, zigzag_hold_distance=0.4)
    assert m.step(0.0, 1.0, zigzag=1) == 0.3
    assert m.step(5.0, 1.0, zigzag=2) == 0.3
    assert m.step(5.5, 1.0) is None
