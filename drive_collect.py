#!/usr/bin/env python3
"""카메라 원본 영상과 BEV 영상을 표시합니다.

q 또는 ESC를 누르거나 창을 닫으면 종료됩니다.
camera_node가 실행 중이면 카메라를 점유할 수 있으므로 먼저 종료하세요.
"""

import argparse

import cv2
import numpy as np


WINDOW = 'Camera'
BEV_WINDOW = 'BEV'


def bev_matrix(width, height):
    """기존 drive_record.py와 같은 원근 변환을 사용한다."""
    source = np.float32([
        (int(width * 0.30), int(height * 0.55)),
        (int(width * 0.70), int(height * 0.55)),
        (width - 1, height - 1),
        (0, height - 1),
    ])
    bev_width = int(width * 0.60)
    x_min = int((width - bev_width) / 2)
    destination = np.float32([
        (x_min, 0),
        (x_min + bev_width, 0),
        (x_min + bev_width, height),
        (x_min, height),
    ])
    return cv2.getPerspectiveTransform(source, destination)


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', default='/dev/video0',
                        help='카메라 장치 (기본: /dev/video0)')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=int, default=30)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)

    cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(
            f'카메라 {args.camera}를 열 수 없습니다. '
            'camera_node가 실행 중이면 먼저 종료하세요.'
        )

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow(BEV_WINDOW, cv2.WINDOW_AUTOSIZE)
    bev_size = None
    bev_mat = None
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                raise SystemExit('카메라 프레임을 읽을 수 없습니다.')

            h, w = frame.shape[:2]
            if bev_size != (w, h):
                bev_size = (w, h)
                bev_mat = bev_matrix(w, h)
                cv2.moveWindow(WINDOW, 0, 0)
                cv2.moveWindow(BEV_WINDOW, w + 20, 0)
            bev = cv2.warpPerspective(frame, bev_mat, bev_size)
            cv2.imshow(WINDOW, frame)
            cv2.imshow(BEV_WINDOW, bev)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break
            # GTK 등에서는 표시 상태 조회를 지원하지 않아 -1을 반환한다.
            # 0(닫힘)이 확인된 경우에만 종료한다.
            if any(cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) == 0
                   for window in (WINDOW, BEV_WINDOW)):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
