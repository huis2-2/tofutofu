"""카메라 픽셀 → 지면 좌표 (BEV) 변환.

차량 좌표계 (base_link): 원점 = 뒷바퀴 축 중앙 바닥, x 앞(+), y 왼쪽(+), 단위 m.

두 가지 방법:
1) 캘리브레이션 파일 (tools/calibrate_camera.py, tools/calibrate_ground.py 결과)
   K, D (렌즈 왜곡), H (왜곡 보정된 픽셀 → 지면 m) 를 그대로 사용
2) 파일이 없으면 카메라 높이·피치·화각으로 핀홀 모델을 만들어 근사 (왜곡 보정 X)

마스크 해상도는 고정이라 **모든 마스크 픽셀의 지면 좌표를 시작할 때 한 번 계산**해 두고
(lookup table) 매 프레임에는 인덱싱만 한다 → 왜곡 보정 + BEV 가 사실상 0 ms.
"""

import math

import numpy as np
import yaml


def pinhole_K(width, height, hfov_deg):
    fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    return np.array([[fx, 0.0, width / 2.0], [0.0, fx, height / 2.0], [0.0, 0.0, 1.0]])


def ground_to_image_homography(K, cam_x, cam_y, cam_h, pitch_deg, yaw_deg=0.0):
    """지면(z=0) 점 [X, Y, 1] → 픽셀 [u, v, 1] 호모그래피. pitch 는 아래로 숙인 각 (+)."""
    th = math.radians(pitch_deg)
    ps = math.radians(yaw_deg)
    # 카메라 축 (차량 좌표계): forward, right, down
    fwd = np.array([math.cos(th) * math.cos(ps), math.cos(th) * math.sin(ps), -math.sin(th)])
    right = np.array([math.sin(ps), -math.cos(ps), 0.0])
    down = np.cross(fwd, right)
    R = np.vstack([right, down, fwd])
    C = np.array([cam_x, cam_y, cam_h])
    t = -R @ C
    return K @ np.column_stack([R[:, 0], R[:, 1], t])


class GroundModel:
    """원본 이미지 픽셀 (u, v) → 지면 (x, y)."""

    def __init__(self, image_width=640, image_height=480, calib_file='',
                 cam_x=0.25, cam_y=0.0, cam_h=0.20, pitch_deg=25.0, hfov_deg=100.0):
        self.width = image_width
        self.height = image_height
        self.K = None
        self.D = None
        self.fisheye = False
        if calib_file:
            with open(calib_file) as f:
                c = yaml.safe_load(f)
            self.width = int(c.get('image_width', image_width))
            self.height = int(c.get('image_height', image_height))
            if 'K' in c:
                self.K = np.array(c['K'], np.float64).reshape(3, 3)
                self.D = np.array(c.get('D', [0, 0, 0, 0, 0]), np.float64).ravel()
                self.fisheye = bool(c.get('fisheye', False))
            self.H = np.array(c['H'], np.float64).reshape(3, 3)  # 보정 픽셀 → 지면
        else:
            K = pinhole_K(self.width, self.height, hfov_deg)
            self.H = np.linalg.inv(ground_to_image_homography(K, cam_x, cam_y, cam_h, pitch_deg))

    def undistort(self, uv):
        if self.K is None:
            return uv
        import cv2
        pts = uv.reshape(-1, 1, 2).astype(np.float64)
        if self.fisheye:
            out = cv2.fisheye.undistortPoints(pts, self.K, self.D[:4], P=self.K)
        else:
            out = cv2.undistortPoints(pts, self.K, self.D, P=self.K)
        return out.reshape(-1, 2)

    def image_to_ground(self, uv):
        """uv: (N, 2) 원본 픽셀 → (N, 2) 지면 [m]. 지평선 위 점은 NaN."""
        uv = self.undistort(np.asarray(uv, np.float64))
        p = np.column_stack([uv, np.ones(len(uv))]) @ self.H.T
        w = p[:, 2]
        with np.errstate(divide='ignore', invalid='ignore'):
            xy = p[:, :2] / w[:, None]
        # 지평선 위 픽셀은 w 부호가 바뀐다. 화면 맨 아래 가운데(확실한 지면)와 부호 비교
        ref = self.H @ np.array([*self.undistort(
            np.array([[self.width / 2.0, self.height - 1.0]]))[0], 1.0])
        xy[np.sign(w) != np.sign(ref[2])] = np.nan
        xy[~np.isfinite(xy).all(axis=1)] = np.nan
        return xy

    def ground_to_image(self, xy):
        """지면 → 왜곡 보정된 픽셀 (디버그 그림용)."""
        Hinv = np.linalg.inv(self.H)
        p = np.column_stack([np.asarray(xy, np.float64), np.ones(len(xy))]) @ Hinv.T
        return p[:, :2] / p[:, 2:3]

    def build_lut(self, mask_width, mask_height, x_min, x_max, y_max):
        """마스크 픽셀마다 지면 좌표 LUT. 범위 밖은 NaN."""
        us = (np.arange(mask_width) + 0.5) * self.width / mask_width
        vs = (np.arange(mask_height) + 0.5) * self.height / mask_height
        uu, vv = np.meshgrid(us, vs)
        xy = self.image_to_ground(np.column_stack([uu.ravel(), vv.ravel()]))
        bad = ~((xy[:, 0] >= x_min) & (xy[:, 0] <= x_max) & (np.abs(xy[:, 1]) <= y_max))
        xy[bad] = np.nan
        return xy.reshape(mask_height, mask_width, 2).astype(np.float32)
