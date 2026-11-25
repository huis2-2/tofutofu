#!/usr/bin/env python3
"""아이폰 화면에 띄울 캘리브레이션 이미지 만들기 (기본: iPhone 16, 1179x2556, 460 ppi).

  python3 make_phone_pattern.py            # → phone_checker.png, phone_marker.png

phone_checker.png : 렌즈 왜곡 캘리브레이션용 체커보드 (calibrate_camera.py)
                    기본 6x15 칸 → 내부 코너 5x14  (--cols 5 --rows 14 로 사용)
phone_marker.png  : 바닥 매핑용 흰 화면 (calibrate_ground_phone.py)

아이폰에서: AirDrop 으로 받아 사진 앱에서 열기 → 화면 한 번 탭해서 UI 숨기기 →
밝기 최대, 자동 잠금 끔, True Tone/Night Shift 는 상관없음. 확대하지 말 것.
"""

import argparse

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--width', type=int, default=1179, help='화면 가로 픽셀 (iPhone 16: 1179)')
    ap.add_argument('--height', type=int, default=2556, help='화면 세로 픽셀 (iPhone 16: 2556)')
    ap.add_argument('--ppi', type=float, default=460.0, help='iPhone 16: 460')
    ap.add_argument('--squares-x', type=int, default=6, help='가로 칸 수')
    ap.add_argument('--squares-y', type=int, default=15, help='세로 칸 수')
    ap.add_argument('--square-px', type=int, default=150)
    args = ap.parse_args()

    w, h, s = args.width, args.height, args.square_px
    bw, bh = args.squares_x * s, args.squares_y * s
    if bw > w - 100 or bh > h - 200:
        raise SystemExit('패턴이 화면보다 큽니다 (가장자리 흰 여백 필요). --square-px 를 줄이세요.')
    img = np.full((h, w), 255, np.uint8)
    x0, y0 = (w - bw) // 2, (h - bh) // 2
    for j in range(args.squares_y):
        for i in range(args.squares_x):
            if (i + j) % 2 == 0:
                img[y0 + j * s:y0 + (j + 1) * s, x0 + i * s:x0 + (i + 1) * s] = 0
    cv2.imwrite('phone_checker.png', img)
    cv2.imwrite('phone_marker.png', np.full((h, w), 255, np.uint8))
    mm = s * 25.4 / args.ppi
    print(f'phone_checker.png: {args.squares_x}x{args.squares_y} 칸, 한 칸 {mm:.2f} mm')
    print(f'  calibrate_camera.py 옵션: --cols {args.squares_x - 1} --rows {args.squares_y - 1} '
          f'--square {mm / 1000:.5f}')
    print('phone_marker.png: 흰 화면 (바닥 매핑용)')


if __name__ == '__main__':
    main()
