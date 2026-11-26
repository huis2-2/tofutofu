import math

from check_bag import analyze, coverage, to_markdown


def run(fps=30, dur=60.0, speed=0.8, drop=None, steer_amp=150, throttle=1820):
    cam = [i / fps for i in range(int(dur * fps))]
    if drop:
        cam = [t for t in cam if not (drop[0] <= t < drop[1])]
    lab = [i / 50 for i in range(int(dur * 50))]
    times = {'/camera/image_raw/compressed': cam, '/drive/steering_us': lab, '/drive/throttle_us': lab,
             '/encoder/speed': lab, '/encoder/distance': lab}
    values = {'/drive/steering_us': [(t, 1660 + steer_amp * math.sin(t)) for t in lab],
              '/drive/throttle_us': [(t, throttle) for t in lab],
              '/encoder/speed': [(t, speed) for t in lab],
              '/encoder/distance': [(t, speed * t) for t in lab]}
    return analyze(times, values, 0.0, dur)


def test_good():
    r = run()
    assert r['verdict'] == 'OK', r
    assert abs(r['camera_fps'] - 30) < 0.5 and r['label_coverage'] > 0.99


def test_frame_drop_is_caution():
    r = run(drop=(10, 11))
    assert r['verdict'] == 'CAUTION' and any('끊김' in w for w in r['warnings'])


def test_not_moving_is_bad():
    r = run(speed=0.0, throttle=1680)
    assert r['verdict'] == 'BAD'
    assert '❌' in to_markdown('x', r)


def test_straight_only_warns():
    r = run(steer_amp=0)
    assert any('직진' in w for w in r['warnings'])


def test_coverage():
    assert coverage([0.0, 1.0, 2.0], [0.05, 2.5]) == 1 / 3
