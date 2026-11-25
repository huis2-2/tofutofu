"""아이폰 바닥 매핑 도구 합성 테스트: 어안 카메라(실측 자세)로 폰 화면을 그려서 H 정확도 확인.

  cd lane_ws/tools && python3 -m pytest -q test_phone_tools.py
"""

import math

import cv2
import numpy as np

from calibrate_ground_phone import diagonal_center, find_phone, fit_homography

W, H_IMG = 640, 480
K = np.array([[200.0, 0, 320], [0, 200.0, 240], [0, 0, 1]])
D = np.array([0.04, -0.01, 0.005, -0.001])
WB = 0.26
CAM = np.array([0.33, 0.0, 0.115])  # 실측: 앞바퀴에서 7cm 앞, 높이 11.5cm
PITCH = math.radians(57)


def world_to_cam(p):
    fwd = np.array([math.cos(PITCH), 0, -math.sin(PITCH)])
    right = np.array([0, -1.0, 0])
    down = np.cross(fwd, right)
    R = np.vstack([right, down, fwd])
    return (R @ (np.asarray(p) - CAM).T).T


def project(p):
    """지면/공간 점 → (왜곡된 픽셀, 왜곡 보정 픽셀)."""
    c = world_to_cam(p)
    und = (K @ (c / c[:, 2:3]).T).T[:, :2]
    dist, _ = cv2.fisheye.projectPoints(c.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, D.reshape(4, 1))
    return dist.reshape(-1, 2), und


def phone_image(center, yaw, bg):
    img = bg.copy()
    w, h = 0.065, 0.141  # 아이폰 16 화면 크기 (m)
    t = np.linspace(-0.5, 0.5, 30)
    edge = np.concatenate([np.column_stack([t * w, np.full_like(t, -h / 2)]),
                           np.column_stack([np.full_like(t, w / 2), t * h]),
                           np.column_stack([-t * w, np.full_like(t, h / 2)]),
                           np.column_stack([np.full_like(t, -w / 2), -t * h])])
    R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    xy = edge @ R.T + center
    dist, _ = project(np.column_stack([xy, np.zeros(len(xy))]))
    cv2.fillPoly(img, [np.round(dist).astype(np.int32)], (250, 250, 250))
    return img


def test_diagonal_center_under_perspective():
    quad = np.array([[10, 10], [110, 20], [100, 60], [20, 50]], float)
    c = diagonal_center(quad[[2, 0, 3, 1]])  # 순서가 섞여도 됨
    assert np.allclose(c, diagonal_center(quad))


def test_phone_ground_calibration():
    bg = np.full((H_IMG, W, 3), 35, np.uint8)
    rng = np.random.default_rng(0)
    spots = [(WB + x, y) for x in (0.10, 0.20, 0.32, 0.45) for y in (-0.15, 0.0, 0.15)]
    img_pts, ground = [], []
    for gx, gy in spots:
        _, und = project(np.array([[gx, gy, 0.0]]))
        if not (0 < und[0, 0] < W * 2):
            continue
        frame = phone_image(np.array([gx, gy]), rng.uniform(-0.6, 0.6), bg)
        det = find_phone(bg, frame, K, D, True)
        assert det is not None, (gx, gy)
        # 대각선 교점 ≈ 실제 중심의 상. 멀리 있으면 폰이 몇 픽셀이라 px 오차는 커짐 → 지면 오차로 판단
        assert np.hypot(*(det[0] - und[0])) < 10.0, (det[0], und[0])
        img_pts.append(det[0])
        ground.append((gx, gy))
    assert len(img_pts) >= 9
    Hm, err = fit_homography(img_pts, ground)
    print('지면 오차 cm:', np.round(err * 100, 2))
    assert err.max() < 0.01, err  # 맞춘 점들 1cm 이내
    # 맞출 때 안 쓴 점(사이 위치)도 정확한지
    for gx, gy in [(WB + 0.15, 0.08), (WB + 0.38, -0.08)]:
        _, und = project(np.array([[gx, gy, 0.0]]))
        q = Hm @ np.array([und[0, 0], und[0, 1], 1.0])
        e = np.hypot(q[0] / q[2] - gx, q[1] / q[2] - gy)
        print(f'검증점 ({(gx - WB) * 100:.0f}, {gy * 100:.0f}) cm 오차 {e * 100:.2f} cm')
        assert e < 0.01


def test_fisheye_calibrate_with_phone_board():
    """폰 체커보드(내부 5x14, 8.28mm)를 10~25cm 에서 찍은 합성 사진 → K, D 복원."""
    from calibrate_camera import fisheye_calibrate
    cols, rows, sq = 5, 14, 0.00828
    objp = np.zeros((1, cols * rows, 3))
    objp[0, :, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * sq
    rng = np.random.default_rng(3)
    objs, imgs = [], []
    while len(objs) < 25:
        rvec = rng.normal(0, 0.4, 3)
        tvec = np.array([rng.uniform(-0.12, 0.07), rng.uniform(-0.12, 0.0), rng.uniform(0.10, 0.25)])
        ip, _ = cv2.fisheye.projectPoints(objp, rvec, tvec, K, D.reshape(4, 1))
        if (ip[..., 0] < 0).any() or (ip[..., 0] > W - 1).any() or (ip[..., 1] < 0).any() \
                or (ip[..., 1] > H_IMG - 1).any():
            continue
        objs.append(objp)
        imgs.append(ip + rng.normal(0, 0.2, ip.shape))
    rms, K2, D2, used = fisheye_calibrate(objs, imgs, (W, H_IMG))
    assert rms < 0.5 and used >= 20
    assert abs(K2[0, 0] - K[0, 0]) / K[0, 0] < 0.03
