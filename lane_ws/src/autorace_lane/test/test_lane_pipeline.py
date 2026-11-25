"""합성 차선(곡선)을 카메라 모델로 마스크에 그려서 전체 파이프라인을 검증."""


import cv2
import numpy as np

from autorace_lane.control_core import ControlParams, LaneController
from autorace_lane.ground import GroundModel
from autorace_lane.lane_core import DASHED, LaneEstimator, LaneParams, YELLOW

MW, MH = 320, 240


def render(model, lines, width_m=0.025):
    """lines: [(cls, f(x)->y)] 지면 선 → 마스크."""
    mask = np.zeros((MH, MW), np.uint8)
    xs = np.linspace(0.2, 2.5, 200)
    for cls, f in lines:
        left = np.column_stack([xs, f(xs) + width_m / 2])
        right = np.column_stack([xs, f(xs) - width_m / 2])
        poly = np.vstack([left, right[::-1]])
        uv = model.ground_to_image(poly) * [MW / 640, MH / 480]
        cv2.fillPoly(mask, [np.round(uv).astype(np.int32)], cls)
    return mask


def run(mask, model, est, bias=0):
    p = est.p
    lut = model.build_lut(MW, MH, p.x_min - 0.05, p.x_max + 0.05, 1.5)
    vv, uu = np.nonzero(mask)
    xy = lut[vv, uu].astype(np.float64)
    cls = mask[vv, uu]
    ok = np.isfinite(xy[:, 0])
    return est.update(xy[ok], cls[ok], bias)


def model():
    return GroundModel(640, 480, '', cam_x=0.25, cam_h=0.20, pitch_deg=25, hfov_deg=100)


def test_round_trip():
    m = model()
    pts = np.array([[0.5, 0.0], [1.0, 0.3], [0.8, -0.2]])
    uv = m.ground_to_image(pts)
    back = m.image_to_ground(uv)
    assert np.allclose(back, pts, atol=1e-6)
    # 화면 아래는 가깝고 위는 멀다
    assert m.ground_to_image(np.array([[0.5, 0.0]]))[0, 1] > m.ground_to_image(np.array([[1.5, 0.0]]))[0, 1]


def test_straight_offset():
    m = model()
    est = LaneEstimator(LaneParams())
    # 차가 차선 중앙에서 오른쪽으로 0.05 치우침 → 중앙선은 y=+0.05
    mask = render(m, [(YELLOW, lambda x: 0.05 + 0.275 + 0 * x),
                      (YELLOW, lambda x: 0.05 - 0.275 + 0 * x)])
    res = run(mask, m, est)
    assert res.valid and res.left.poly is not None and res.right.poly is not None
    assert abs(res.lateral_error - 0.05) < 0.02, res.lateral_error
    assert abs(res.heading) < 0.05


def test_curve_one_side_and_steer():
    m = model()
    R = 0.9  # 왼쪽으로 도는 커브 (중심 y=+R)
    center = lambda x: R - np.sqrt(np.maximum(R * R - x * x, 0))  # noqa: E731
    est = LaneEstimator(LaneParams())
    outer = lambda x: R - np.sqrt(np.maximum((R + 0.275) ** 2 - x * x, 0))  # noqa: E731
    mask = render(m, [(YELLOW, outer)])  # 오른쪽(바깥) 선만 보임
    res = run(mask, m, est)
    assert res.valid and res.right.poly is not None and res.left.poly is None
    y_true = center(0.55)
    assert abs(res.lateral_error - y_true) < 0.06, (res.lateral_error, y_true)
    ctrl = LaneController(ControlParams())
    v, steer, _ = ctrl.step(0.0, 0.03, True, res.path, res.heading, 0.8)
    assert steer > 0.1  # 왼쪽 조향


def test_dashed_and_lane_change():
    m = model()
    est = LaneEstimator(LaneParams())
    lines = [(YELLOW, lambda x: 0.275 + 0.55 + 0 * x), (DASHED, lambda x: 0.275 + 0 * x),
             (YELLOW, lambda x: -0.275 + 0 * x)]
    mask = render(m, lines)
    res = run(mask, m, est)
    assert abs(res.lateral_error) < 0.03
    assert res.left.dashed_ratio > 0.5 and res.right.dashed_ratio < 0.5
    est.lane_change(+1)
    res = run(mask, m, est)
    assert abs(res.lateral_error - 0.55) < 0.05, res.lateral_error


def test_fork_bias():
    m = model()
    est = LaneEstimator(LaneParams())
    # 갈림길: 왼쪽 길은 왼쪽으로, 오른쪽 길은 오른쪽으로 벌어짐
    lines = [(YELLOW, lambda x: 0.275 + 0.3 * x * x), (YELLOW, lambda x: -0.275 - 0.3 * x * x)]
    mask = render(m, lines)
    res_a = run(mask, m, LaneEstimator(LaneParams()), bias=+1)
    res_b = run(mask, m, LaneEstimator(LaneParams()), bias=-1)
    assert res_a.path[-1, 1] > 0.2 and res_b.path[-1, 1] < -0.2
    del est


def test_crosswalk_distance():
    m = model()
    est = LaneEstimator(LaneParams())
    mask = render(m, [(YELLOW, lambda x: 0.275 + 0 * x), (YELLOW, lambda x: -0.275 + 0 * x)])
    cw = np.array([[1.0, 0.2], [1.0, -0.2], [1.3, -0.2], [1.3, 0.2]])
    uv = m.ground_to_image(cw) * [MW / 640, MH / 480]
    cv2.fillPoly(mask, [np.round(uv).astype(np.int32)], 5)
    res = run(mask, m, est)
    assert abs(res.crosswalk - (1.0 - 0.26)) < 0.04, res.crosswalk


if __name__ == '__main__':
    for k, f in list(globals().items()):
        if k.startswith('test_'):
            f()
            print('ok', k)
