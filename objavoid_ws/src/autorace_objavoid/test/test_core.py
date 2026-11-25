import numpy as np

from autorace_objavoid.core import ObjAvoid, scan_to_points


def box(x, y, n=10):
    return np.column_stack([np.full(n, x), np.linspace(y - 0.1, y + 0.1, n)])


def test_changes_to_free_dashed_side():
    oa = ObjAvoid(prefer=1)
    oa.on_yolo(0.0)
    pts = np.vstack([box(0.8, 0.0), box(0.9, 0.55)])  # 앞, 왼쪽 막힘 → 오른쪽
    v, change = oa.step(0.0, pts, True, True, 1.0)
    assert change == -1 and v == 0.5
    v, change = oa.step(0.1, pts, True, True, 1.0)
    assert change == 0  # cooldown


def test_no_change_over_yellow():
    oa = ObjAvoid()
    oa.on_yolo(0.0)
    pts = box(0.6, 0.0)
    v, change = oa.step(0.0, pts, False, False, 1.0)
    assert change == 0 and v is not None and v <= 0.25


def test_scan_conversion():
    pts = scan_to_points([1.0], 0.0, 0.01, lidar_x=0.2, lidar_yaw=0.0)
    assert np.allclose(pts, [[1.2, 0.0]])
