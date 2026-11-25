import numpy as np

from autorace_cone.core import ConeCourse, clusters


def cone(x, y, r=0.04):
    a = np.linspace(-np.pi / 2, np.pi / 2, 6)  # LiDAR 쪽 반원
    return np.column_stack([x - r * np.cos(a), y + r * np.sin(a)])


def test_clusters_and_steer_between():
    pts = np.vstack([cone(0.6, 0.35), cone(0.6, -0.15), cone(1.1, 0.45), cone(1.1, -0.05)])
    c = clusters(pts)
    assert len(c) == 4
    cc = ConeCourse(require_yolo=False)
    speed, steer = cc.step(0.0, c)
    assert speed > 0 and steer > 0.02  # 콘 중앙이 왼쪽(+0.1) → 왼쪽 조향
