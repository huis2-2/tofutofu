import numpy as np

from autorace_yolo.classes import normalize
from autorace_yolo.core import Confirmer, best_boxes, draw_lane_mask


def test_normalize():
    assert normalize('Dashed_line') == 'dashed_line'
    assert normalize('Go_A') == 'go_a'
    assert normalize('cross-walk') == 'cross_walk'
    assert normalize('slow_down') == 'slow'
    assert normalize('Parking') == 'parking'


def test_mask_priority():
    def fill(mask, pts, v):
        x0, y0 = pts.min(axis=0)
        x1, y1 = pts.max(axis=0)
        mask[y0:y1, x0:x1] = v
    sq = np.array([[0.1, 0.1], [0.5, 0.1], [0.5, 0.5], [0.1, 0.5]])
    m = draw_lane_mask([('yellow_line', sq), ('cross_walk', sq)], 100, 100, fill)
    assert m[30, 30] == 1  # 노란선이 위에 그려짐


def test_confirm_and_best():
    c = Confirmer(['slow', 'cone'], 2)
    assert not c.update({'slow'})['slow']
    assert c.update({'slow'})['slow']
    b = best_boxes([('slow', 0.9, 0, 0, 0.1, 0.1), ('slow', 0.8, 0, 0, 0.3, 0.3)], {'default': 0})
    assert b['slow'][5] == 0.3
