from autorace_mux.core import Arbiter


def test_priority_and_timeout():
    a = Arbiter(timeout=0.25)
    a.receive('lane', 1.0, 0.1, 0.0)
    a.receive('slow', 0.4, 0.1, 0.0)
    assert a.select(0.1)[0] == 'slow'
    a.receive('lane', 1.0, 0.1, 0.3)
    assert a.select(0.4) == ('lane', 1.0, 0.1)
    a.receive('crosswalk', 0.0, 0.1, 0.4)
    assert a.select(0.45)[0:2] == ('crosswalk', 0.0)
    assert a.select(0.45, disabled={'crosswalk'})[0] == 'lane'
    assert a.select(1.0)[0] == 'none'
