#!/usr/bin/env python3
"""바닥에 체커보드를 놓고 '왜곡 보정 픽셀 → 지면(m)' 호모그래피 H 를 구한다.

준비: 차를 트랙에 똑바로 세우고 체커보드를 차 앞 바닥에 평평하게, 차 진행방향과 나란히 둔다.
      체커보드 '차에 가장 가까운 줄, 오른쪽 끝' 내부 코너의 위치를 줄자로 잰다:
        --origin-x : 뒷바퀴 축 → 그 코너 까지 앞쪽 거리 (m)
        --origin-y : 차 중심선 → 그 코너 까지 왼쪽(+) 거리 (m)  (오른쪽이면 음수)
실행 (camera_node 끄고):
  python3 calibrate_ground.py --calib ../src/autorace_lane/config/camera_calib.yaml \
      --cols 9 --rows 6 --square 0.025 --origin-x 0.45 --origin-y -0.10
  화면에서 코너가 잡히면 s = 저장. 결과의 H 가 같은 yaml 에 추가된다.
  lane.yaml 의 calib_file 에 이 yaml 경로를 넣으면 끝.
"""

import argparse

import cv2
import numpy as np
import yaml


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--calib', required=True)
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--image', default='', help='카메라 대신 사진 파일 사용')
    ap.add_argument('--cols', type=int, default=9)
    ap.add_argument('--rows', type=int, default=6)
    ap.add_argument('--square', type=float, default=0.025)
    ap.add_argument('--origin-x', type=float, required=True)
    ap.add_argument('--origin-y', type=float, required=True)
    args = ap.parse_args()

    with open(args.calib) as f:
        calib = yaml.safe_load(f) or {}
    K = np.array(calib['K']).reshape(3, 3) if 'K' in calib else None
    D = np.array(calib.get('D', [])) if 'K' in calib else None
    fisheye = calib.get('fisheye', False)

    def undist(pts):
        if K is None:
            return pts.reshape(-1, 2)
        if fisheye:
            return cv2.fisheye.undistortPoints(pts, K, D[:4], P=K).reshape(-1, 2)
        return cv2.undistortPoints(pts, K, D, P=K).reshape(-1, 2)

    cap = None if args.image else cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if cap is not None:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    pattern = (args.cols, args.rows)
    while True:
        frame = cv2.imread(args.image) if args.image else cap.read()[1]
        if frame is None:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ok, corners = cv2.findChessboardCorners(gray, pattern, None)
        view = frame.copy()
        if ok:
            corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                                       (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
            cv2.drawChessboardCorners(view, pattern, corners, ok)
        cv2.imshow('ground calib (s=save, q=quit)', view)
        k = cv2.waitKey(30) & 0xFF
        if k in (ord('q'), 27):
            return
        if k == ord('s') and ok:
            break

    uv = undist(corners.astype(np.float64))
    # 코너 순서를 지면 좌표에 맞춘다: 이미지에서 가장 아래(가까운) 줄이 x 최소
    pts = uv.reshape(args.rows, args.cols, 2)
    if pts[0, :, 1].mean() < pts[-1, :, 1].mean():  # 첫 줄이 위(먼 쪽)이면 뒤집기
        pts = pts[::-1]
    if pts[0, 0, 0] > pts[0, -1, 0]:  # 각 줄을 오른쪽→왼쪽이 아니라 왼쪽→오른쪽으로 정렬했는지
        pts = pts[:, ::-1]
    # 이미지 왼쪽 = 지면 +y. 오른쪽 끝 코너가 원점
    ground = np.zeros((args.rows, args.cols, 2))
    for r in range(args.rows):
        for c in range(args.cols):
            ground[r, c] = (args.origin_x + r * args.square,
                            args.origin_y + (args.cols - 1 - c) * args.square)
    H, _ = cv2.findHomography(pts.reshape(-1, 2), ground.reshape(-1, 2))
    err = cv2.perspectiveTransform(pts.reshape(-1, 1, 2), H).reshape(-1, 2) - ground.reshape(-1, 2)
    print('지면 오차 평균 %.4f m, 최대 %.4f m' % (np.abs(err).mean(), np.abs(err).max()))
    calib.update({'image_width': frame.shape[1], 'image_height': frame.shape[0],
                  'H': H.ravel().tolist()})
    with open(args.calib, 'w') as f:
        yaml.safe_dump(calib, f)
    print('저장:', args.calib)


if __name__ == '__main__':
    main()
