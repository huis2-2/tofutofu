#!/usr/bin/env python3
"""체커보드로 렌즈 왜곡(내부 파라미터) 캘리브레이션.

1) 사진 찍기 (camera_node 끄고):  python3 calibrate_camera.py capture --out calib_imgs
   스페이스 = 저장, q = 종료. 체커보드를 화면 가장자리/기울기 다양하게 20장 이상
2) 계산: python3 calibrate_camera.py solve calib_imgs --cols 9 --rows 6 --square 0.025 \
         --out ../src/autorace_lane/config/camera_calib.yaml [--fisheye]
   cols/rows = 내부 코너 개수, square = 한 칸 크기 (m)
"""

import argparse
import glob
import os

import cv2
import numpy as np
import yaml


def capture(args):
    os.makedirs(args.out, exist_ok=True)
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    n = len(glob.glob(os.path.join(args.out, '*.png')))
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        cv2.imshow('capture (space=save, q=quit)', frame)
        k = cv2.waitKey(1) & 0xFF
        if k == ord(' '):
            cv2.imwrite(os.path.join(args.out, f'{n:03d}.png'), frame)
            n += 1
            print('saved', n)
        elif k in (ord('q'), 27):
            break
    cap.release()
    cv2.destroyAllWindows()


def solve(args):
    pattern = (args.cols, args.rows)
    objp = np.zeros((args.cols * args.rows, 1, 3), np.float32)
    objp[:, 0, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square
    objs, imgs, size = [], [], None
    for f in sorted(glob.glob(os.path.join(args.dir, '*.png')) + glob.glob(os.path.join(args.dir, '*.jpg'))):
        gray = cv2.cvtColor(cv2.imread(f), cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]
        ok, corners = cv2.findChessboardCorners(gray, pattern, None)
        if not ok:
            print('코너 못 찾음:', f)
            continue
        corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                                   (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
        objs.append(objp)
        imgs.append(corners)
    print(f'{len(objs)} 장 사용')
    if args.fisheye:
        K = np.zeros((3, 3))
        D = np.zeros((4, 1))
        flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
        rms, K, D, _, _ = cv2.fisheye.calibrate(objs, imgs, size, K, D, flags=flags)
    else:
        rms, K, D, _, _ = cv2.calibrateCamera([o.reshape(-1, 3) for o in objs], imgs, size, None, None)
    print('RMS 재투영 오차', rms)
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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    c = sub.add_parser('capture')
    c.add_argument('--device', default='/dev/video0')
    c.add_argument('--out', default='calib_imgs')
    s = sub.add_parser('solve')
    s.add_argument('dir')
    s.add_argument('--cols', type=int, default=9)
    s.add_argument('--rows', type=int, default=6)
    s.add_argument('--square', type=float, default=0.025)
    s.add_argument('--fisheye', action='store_true')
    s.add_argument('--out', default='camera_calib.yaml')
    args = ap.parse_args()
    capture(args) if args.cmd == 'capture' else solve(args)


if __name__ == '__main__':
    main()
