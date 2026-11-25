#!/usr/bin/env python3
"""렌즈 왜곡(내부 파라미터) 캘리브레이션. 아이폰 화면 체커보드 또는 종이 체커보드.

0) 패턴: python3 make_phone_pattern.py → phone_checker.png 를 아이폰에 띄움 (내부 코너 5x14)
1) 찍기 (camera_node 끄고, 차 카메라 그대로):
     python3 calibrate_camera.py capture --cols 5 --rows 14 --out calib_imgs
   - 화면에 코너가 잡히면 초록색으로 표시됨. 스페이스 = 저장 (잡혔을 때만), q = 끝
   - 폰을 카메라에서 10~25cm 에 두고 화면 가운데, 네 귀퉁이, 가장자리, 기울여서(30~45도)
     골고루 25장 이상. 오른쪽 위 격자가 다 초록이 되게 (어안은 가장자리가 중요)
   - 폰이 흔들리면 흐려지므로 멈춘 상태에서 저장
2) 계산:
     python3 calibrate_camera.py solve calib_imgs --cols 5 --rows 14 --square 0.00828 --fisheye \\
         --out ../src/autorace_lane/config/camera_calib.yaml
   (칸 크기 --square 는 렌즈 왜곡 결과에 영향 없음)
"""

import argparse
import glob
import os

import cv2
import numpy as np
import yaml

GRID = 4  # 커버리지 표시 격자


def find_corners(gray, pattern):
    """SB 검출기 (화면/모아레에 강함) → 실패하면 기존 방식."""
    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    ok, corners = cv2.findChessboardCornersSB(gray, pattern, flags=flags)
    if ok:
        return True, corners.astype(np.float32)
    ok, corners = cv2.findChessboardCorners(
        gray, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if ok:
        corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                                   (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
    return ok, corners


def open_camera(device):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        raise SystemExit(f'{device} 를 열 수 없습니다 (camera_node 가 켜져 있으면 끄세요)')
    return cap


def capture(args):
    os.makedirs(args.out, exist_ok=True)
    cap = open_camera(args.device)
    pattern = (args.cols, args.rows)
    n = len(glob.glob(os.path.join(args.out, '*.png')))
    covered = np.zeros((GRID, GRID), bool)
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = find_corners(gray, pattern)
        view = frame.copy()
        if found:
            cv2.drawChessboardCorners(view, pattern, corners, found)
        h, w = gray.shape
        for (gy, gx), c in np.ndenumerate(covered):
            x0, y0 = w - 70 + gx * 15, 5 + gy * 15
            cv2.rectangle(view, (x0, y0), (x0 + 13, y0 + 13), (0, 200, 0) if c else (80, 80, 80), -1)
        cv2.putText(view, f'{n} saved  {"FOUND" if found else "-"}', (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if found else (0, 0, 255), 2)
        cv2.imshow('capture (space=save, q=quit)', view)
        k = cv2.waitKey(1) & 0xFF
        if k == ord(' ') and found:
            cv2.imwrite(os.path.join(args.out, f'{n:03d}.png'), frame)
            n += 1
            for x, y in corners.reshape(-1, 2):
                covered[min(GRID - 1, int(y / h * GRID)), min(GRID - 1, int(x / w * GRID))] = True
            print('saved', n)
        elif k in (ord('q'), 27):
            break
    cap.release()
    cv2.destroyAllWindows()


def fisheye_calibrate(objs, imgs, size):
    """초기 초점거리 추정 + 문제 있는 사진을 자동으로 빼면서 fisheye 캘리브레이션."""
    w, h = size
    K0 = np.array([[w / 3.0, 0, w / 2.0], [0, w / 3.0, h / 2.0], [0, 0, 1]])
    flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
             | cv2.fisheye.CALIB_USE_INTRINSIC_GUESS)
    crit = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 100, 1e-6)

    def run(idx):
        return cv2.fisheye.calibrate([objs[i] for i in idx], [imgs[i] for i in idx], size,
                                     K0.copy(), np.zeros((4, 1)), flags=flags, criteria=crit)
    try:
        return run(range(len(objs)))[:3] + (len(objs),)
    except cv2.error:
        pass
    keep = []
    for i in range(len(objs)):  # 하나씩 넣어 보며 실패시키는 사진은 제외
        try:
            if len(keep) >= 3:
                run(keep + [i])
            keep.append(i)
        except cv2.error:
            print(f'  사진 {i} 제외 (수치 문제)')
    return run(keep)[:3] + (len(keep),)


def solve(args):
    pattern = (args.cols, args.rows)
    objp = np.zeros((1, args.cols * args.rows, 3), np.float64)
    objp[0, :, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square
    objs, imgs, size = [], [], None
    files = sorted(glob.glob(os.path.join(args.dir, '*.png')) + glob.glob(os.path.join(args.dir, '*.jpg')))
    for f in files:
        gray = cv2.cvtColor(cv2.imread(f), cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]
        ok, corners = find_corners(gray, pattern)
        if not ok:
            print('코너 못 찾음:', f)
            continue
        objs.append(objp)
        imgs.append(corners.reshape(1, -1, 2).astype(np.float64))
    if len(objs) < 8:
        raise SystemExit(f'사용 가능한 사진이 {len(objs)}장뿐입니다 (15장 이상 권장)')
    if args.fisheye:
        rms, K, D, used = fisheye_calibrate(objs, imgs, size)
    else:
        rms, K, D, _, _ = cv2.calibrateCamera([o.reshape(-1, 3).astype(np.float32) for o in objs],
                                              [i.reshape(-1, 1, 2).astype(np.float32) for i in imgs],
                                              size, None, None)
        used = len(objs)
    print(f'{used} 장 사용, RMS 재투영 오차 {rms:.3f} px (0.5 이하면 좋음, 1 넘으면 다시 찍기)')
    print('K =', np.round(K, 2).tolist())
    print('D =', np.round(np.asarray(D).ravel(), 4).tolist())
    data = {'image_width': size[0], 'image_height': size[1], 'fisheye': bool(args.fisheye),
            'K': K.ravel().tolist(), 'D': np.asarray(D).ravel().tolist()}
    if os.path.exists(args.out):
        with open(args.out) as f:
            old = yaml.safe_load(f) or {}
        old.update(data)
        data = old
    with open(args.out, 'w') as f:
        yaml.safe_dump(data, f)
    print('저장:', args.out)
    if args.preview:
        img = cv2.imread(files[0])
        if args.fisheye:
            P = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, size, np.eye(3), balance=0.5)
            m1, m2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), P, size, cv2.CV_16SC2)
            out = cv2.remap(img, m1, m2, cv2.INTER_LINEAR)
        else:
            out = cv2.undistort(img, K, D)
        cv2.imshow('before', img)
        cv2.imshow('after (직선이 곧게 보이면 성공)', out)
        cv2.waitKey(0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    c = sub.add_parser('capture')
    c.add_argument('--device', default='/dev/video0')
    c.add_argument('--out', default='calib_imgs')
    c.add_argument('--cols', type=int, default=5, help='내부 코너 가로 개수')
    c.add_argument('--rows', type=int, default=14, help='내부 코너 세로 개수')
    s = sub.add_parser('solve')
    s.add_argument('dir')
    s.add_argument('--cols', type=int, default=5)
    s.add_argument('--rows', type=int, default=14)
    s.add_argument('--square', type=float, default=0.00828, help='칸 크기 m (왜곡 결과엔 영향 없음)')
    s.add_argument('--fisheye', action='store_true', help='어안(광각) 모델. 우리 카메라는 이걸로')
    s.add_argument('--preview', action='store_true', help='보정 전/후 사진 보기')
    s.add_argument('--out', default='camera_calib.yaml')
    args = ap.parse_args()
    capture(args) if args.cmd == 'capture' else solve(args)


if __name__ == '__main__':
    main()
