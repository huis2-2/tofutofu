#!/usr/bin/env python3
"""아이폰을 '움직이는 점 마커'로 써서 바닥 매핑(왜곡 보정 픽셀 → 지면 m, H)을 구한다.

왜 체커보드를 바닥에 안 깔고 이렇게 하나: 폰 화면은 작아서 멀리 놓으면 코너가 안 잡힌다.
대신 흰 화면(phone_marker.png)을 띄운 폰을 바닥 여러 곳에 옮겨 놓고, 화면 사각형의
**대각선 교점**(원근이 있어도 정확한 중심)을 찾아 줄자로 잰 위치와 짝지어 H 를 맞춘다.

준비
  - calibrate_camera.py 로 렌즈 왜곡(K, D) 먼저 → camera_calib.yaml
  - 차를 직선 구간에 차선과 나란히, 바퀴 똑바로 세우고 움직이지 않게
  - 바닥에 테이프로 점 9~12개 표시: 앞바퀴 축 중심 기준
      앞(x): 10, 25, 40, 60 cm ... (화면에 보이는 범위 안, 가까운 곳부터 먼 곳까지 골고루)
      옆(y): 왼쪽 +, 오른쪽 - (예: -25, 0, +25 cm)
  - 아이폰: phone_marker.png 를 사진 앱으로 띄우고 밝기 최대, 자동 잠금 끔

실행 (camera_node 끄고)
  python3 calibrate_ground_phone.py --calib ../src/autorace_lane/config/camera_calib.yaml
  b : 배경 저장 (폰·손 없이!)  ← 처음 한 번, 조명이 바뀌면 다시
  c : 폰을 표시점 위에 화면 가운데가 오게 눕혀 놓고, 손 치운 뒤 누름 → 터미널에 "x y" (cm) 입력
  u : 마지막 점 취소     f : 계산·저장     v : 확인 화면 (예상 차선/거리 눈금 겹쳐 보기)    q : 끝
"""

import argparse

import cv2
import numpy as np
import yaml


# ---------- 계산 함수 (테스트 가능) ----------

def undistort_pts(pts, K, D, fisheye):
    pts = np.asarray(pts, np.float64).reshape(-1, 1, 2)
    if K is None:
        return pts.reshape(-1, 2)
    if fisheye:
        return cv2.fisheye.undistortPoints(pts, K, D[:4].reshape(4, 1), P=K).reshape(-1, 2)
    return cv2.undistortPoints(pts, K, D, P=K).reshape(-1, 2)


def distort_pts(pts, K, D, fisheye):
    """왜곡 보정된 픽셀 → 원본(왜곡된) 픽셀 (확인 화면 그리기용)."""
    pts = np.asarray(pts, np.float64).reshape(-1, 2)
    if K is None:
        return pts
    norm = (np.linalg.inv(K) @ np.column_stack([pts, np.ones(len(pts))]).T).T[:, :2]
    if fisheye:
        return cv2.fisheye.distortPoints(norm.reshape(-1, 1, 2), K, D[:4].reshape(4, 1)).reshape(-1, 2)
    obj = np.column_stack([norm, np.ones(len(norm))])
    out, _ = cv2.projectPoints(obj, np.zeros(3), np.zeros(3), K, D)
    return out.reshape(-1, 2)


def diagonal_center(quad):
    """사각형 4점 → 대각선 교점 (원근 변환에서도 실제 중심의 상)."""
    q = order_quad(np.asarray(quad, np.float64))
    p1, p3, p2, p4 = q[0], q[2], q[1], q[3]
    d1, d2 = p3 - p1, p4 - p2
    A = np.array([d1, -d2]).T
    t = np.linalg.solve(A, p2 - p1)
    return p1 + t[0] * d1


def order_quad(q):
    """중심 기준 각도 순 정렬 (대각선 쌍이 0-2, 1-3 이 되게)."""
    c = q.mean(axis=0)
    ang = np.arctan2(q[:, 1] - c[1], q[:, 0] - c[0])
    return q[np.argsort(ang)]


def find_phone(background, frame, K=None, D=None, fisheye=False, min_area=40, diff_thresh=60):
    """배경과 비교해 새로 생긴 밝은 사각형(폰 화면) → (왜곡 보정 중심, 원본 윤곽) 또는 None."""
    g0 = cv2.cvtColor(background, cv2.COLOR_BGR2GRAY).astype(np.int16)
    g1 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.int16)
    mask = ((g1 - g0) > diff_thresh).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None
    cnt = max(contours, key=cv2.contourArea)
    if cv2.contourArea(cnt) < min_area:
        return None
    und = undistort_pts(cnt.reshape(-1, 2), K, D, fisheye).astype(np.float32)
    hull = cv2.convexHull(und)
    quad = cv2.approxPolyDP(hull, 0.04 * cv2.arcLength(hull, True), True).reshape(-1, 2)
    if len(quad) != 4:
        quad = cv2.boxPoints(cv2.minAreaRect(und))
    return diagonal_center(quad), cnt


def fit_homography(img_pts, ground_pts):
    img_pts = np.asarray(img_pts, np.float64)
    ground_pts = np.asarray(ground_pts, np.float64)
    if len(img_pts) >= 8:
        H, _ = cv2.findHomography(img_pts, ground_pts, cv2.RANSAC, 0.03)
    else:
        H, _ = cv2.findHomography(img_pts, ground_pts, 0)
    p = np.column_stack([img_pts, np.ones(len(img_pts))]) @ H.T
    err = p[:, :2] / p[:, 2:3] - ground_pts
    return H, np.hypot(err[:, 0], err[:, 1])


# ---------- 실행 ----------

def draw_check(view, H, K, D, fisheye, wheelbase, lane_width):
    """지면 선(차선 ±폭/2, 앞바퀴 기준 10cm 간격 눈금)을 원본 화면에 투영."""
    Hinv = np.linalg.inv(H)

    ref = Hinv @ np.array([wheelbase + 0.1, 0.0, 1.0])  # 확실히 보이는 점의 부호

    def to_img(xy):
        p = np.column_stack([xy, np.ones(len(xy))]) @ Hinv.T
        p = p[np.sign(p[:, 2]) == np.sign(ref[2])]  # 카메라 뒤(지평선 위) 점 제외
        return distort_pts(p[:, :2] / p[:, 2:3], K, D, fisheye), len(p)

    xs = np.linspace(wheelbase, wheelbase + 1.5, 60)
    for y, col in ((lane_width / 2, (0, 255, 255)), (-lane_width / 2, (0, 255, 255)), (0.0, (255, 0, 255))):
        pts, n = to_img(np.column_stack([xs, np.full_like(xs, y)]))
        if n < 2:
            continue
        cv2.polylines(view, [np.round(pts).astype(np.int32)], False, col, 1)
    for d in np.arange(0.1, 1.51, 0.1):
        ys = np.linspace(-0.4, 0.4, 20)
        pts, n = to_img(np.column_stack([np.full_like(ys, wheelbase + d), ys]))
        if n < 2:
            continue
        cv2.polylines(view, [np.round(pts).astype(np.int32)], False, (0, 200, 0), 1)
        cv2.putText(view, f'{int(round(d * 100))}', tuple(np.round(pts[-1]).astype(int)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 200, 0), 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--calib', required=True, help='calibrate_camera.py 결과 yaml (H 가 여기에 추가됨)')
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--wheelbase', type=float, default=0.26, help='뒷바퀴 축 → 앞바퀴 축 (m)')
    ap.add_argument('--lane-width', type=float, default=0.55)
    ap.add_argument('--diff-thresh', type=int, default=60)
    args = ap.parse_args()

    with open(args.calib) as f:
        calib = yaml.safe_load(f) or {}
    K = np.array(calib['K']).reshape(3, 3) if 'K' in calib else None
    D = np.array(calib.get('D', [0, 0, 0, 0])) if K is not None else None
    fisheye = bool(calib.get('fisheye', False))
    if K is None:
        print('경고: K/D 가 없습니다. 렌즈 왜곡 보정 없이 진행 (어안이면 부정확)')

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    background = None
    img_pts, ground_pts, raw_pts = [], [], []
    H = None
    show_check = False
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        view = frame.copy()
        det = find_phone(background, frame, K, D, fisheye, diff_thresh=args.diff_thresh) \
            if background is not None else None
        if det is not None:
            cv2.drawContours(view, [det[1]], -1, (0, 0, 255), 1)
            c = distort_pts(det[0][None], K, D, fisheye)[0]
            cv2.drawMarker(view, tuple(np.round(c).astype(int)), (0, 0, 255), cv2.MARKER_CROSS, 12, 2)
        for p in raw_pts:
            cv2.circle(view, tuple(np.round(p).astype(int)), 4, (0, 255, 0), -1)
        if show_check and H is not None:
            draw_check(view, H, K, D, fisheye, args.wheelbase, args.lane_width)
        msg = ('b=background first' if background is None
               else f'{len(img_pts)} pts  c=add u=undo f=fit v=check q=quit')
        cv2.putText(view, msg, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.imshow('ground calib (phone)', view)
        k = cv2.waitKey(30) & 0xFF
        if k in (ord('q'), 27):
            break
        if k == ord('b'):
            background = frame.copy()
            print('배경 저장. 이제 폰을 표시점에 놓고 c')
        elif k == ord('c'):
            if det is None:
                print('폰을 못 찾음: 배경(b)을 폰 없이 찍었는지, 밝기 최대인지 확인')
                continue
            s = input('이 점의 위치 "x y" (cm, 앞바퀴 축 중심 기준, x 앞 / y 왼쪽+), 엔터=취소: ').strip()
            if not s:
                continue
            try:
                x_cm, y_cm = (float(v) for v in s.replace(',', ' ').split())
            except ValueError:
                print('형식: 25 -10')
                continue
            img_pts.append(det[0])
            raw_pts.append(distort_pts(det[0][None], K, D, fisheye)[0])
            ground_pts.append((args.wheelbase + x_cm / 100.0, y_cm / 100.0))
            print(f'점 {len(img_pts)}: 지면 ({x_cm:.0f}, {y_cm:.0f}) cm')
        elif k == ord('u') and img_pts:
            img_pts.pop()
            ground_pts.pop()
            raw_pts.pop()
            print('마지막 점 취소')
        elif k == ord('f'):
            if len(img_pts) < 4:
                print('점이 4개 이상 필요 (9~12개 권장)')
                continue
            H, err = fit_homography(img_pts, ground_pts)
            for i, e in enumerate(err):
                g = ground_pts[i]
                print(f'  점 {i + 1} ({(g[0] - args.wheelbase) * 100:.0f}, {g[1] * 100:.0f}) cm: '
                      f'오차 {e * 100:.1f} cm')
            print(f'평균 오차 {err.mean() * 100:.1f} cm, 최대 {err.max() * 100:.1f} cm '
                  '(2cm 넘는 점은 u 로 빼고 다시 재기)')
            calib.update({'image_width': frame.shape[1], 'image_height': frame.shape[0],
                          'H': H.ravel().tolist()})
            with open(args.calib, 'w') as f:
                yaml.safe_dump(calib, f)
            print('저장:', args.calib, ' → v 로 확인 (노란선 = 예상 차선, 숫자 = 앞바퀴에서 cm)')
        elif k == ord('v'):
            show_check = not show_check
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
