"""ROS 와 무관한 후처리: 연속 프레임 확인, 차선 마스크 그리기, 결과 정리."""

import numpy as np

from .classes import LANE_DRAW_ORDER, LANE_IDS, SIGN_NAMES


class Confirmer:
    """클래스별 연속 검출 프레임 수를 세서 confirm_frames 이상이면 True."""

    def __init__(self, names, confirm_frames=2):
        self.need = max(1, int(confirm_frames))
        self.count = {n: 0 for n in names}

    def update(self, present):
        out = {}
        for name in self.count:
            self.count[name] = self.count[name] + 1 if name in present else 0
            out[name] = self.count[name] >= self.need
        return out


def draw_lane_mask(polygons, width, height, fill_poly):
    """polygons: [(name, Nx2 정규화 좌표)] → 클래스 값 마스크 (height, width) uint8.

    fill_poly(mask, pts_int32, value) 는 cv2.fillPoly 래퍼 (테스트에서 교체 가능).
    """
    mask = np.zeros((height, width), np.uint8)
    order = {n: i for i, n in enumerate(LANE_DRAW_ORDER)}
    scale = np.array([width, height], np.float32)
    for name, poly in sorted(polygons, key=lambda p: order.get(p[0], -1)):
        if name not in LANE_IDS or poly is None or len(poly) < 3:
            continue
        pts = np.round(np.asarray(poly, np.float32) * scale).astype(np.int32)
        fill_poly(mask, pts, LANE_IDS[name])
    return mask


def best_boxes(dets, min_area):
    """dets: [(name, conf, x1, y1, x2, y2)] (정규화) → {name: 가장 큰 박스}."""
    best = {}
    for d in dets:
        name, conf, x1, y1, x2, y2 = d
        if name not in SIGN_NAMES:
            continue
        area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        if area < min_area.get(name, min_area.get('default', 0.0)):
            continue
        if name not in best or area > best[name][1]:
            best[name] = (d, area)
    return {k: v[0] for k, v in best.items()}
