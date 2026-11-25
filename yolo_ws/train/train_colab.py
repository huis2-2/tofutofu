"""Colab 에서 YOLO11n-seg 학습 (Roboflow huieuns-workspace/tofu_v3).

Colab 셀에서:
  !pip install -q ultralytics roboflow
  import os; os.environ['ROBOFLOW_API_KEY'] = '...'   # 키는 코드/깃에 넣지 말 것
  !python train_colab.py --version 1

학습이 끝나면 runs/segment/tofu/weights/best.pt 를 받아서
Jetson 의 yolo_ws/model/best.pt 로 복사 → README 의 export 명령으로 엔진 변환.
"""

import argparse
import os

from roboflow import Roboflow
from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--workspace', default='huieuns-workspace')
    ap.add_argument('--project', default='tofu_v3')
    ap.add_argument('--version', type=int, required=True)
    ap.add_argument('--model', default='yolo11n-seg.pt')
    ap.add_argument('--epochs', type=int, default=150)
    ap.add_argument('--batch', type=int, default=16)
    args = ap.parse_args()

    rf = Roboflow(api_key=os.environ['ROBOFLOW_API_KEY'])
    ds = rf.workspace(args.workspace).project(args.project).version(args.version).download('yolov11')

    model = YOLO(args.model)
    model.train(
        data=os.path.join(ds.location, 'data.yaml'),
        imgsz=640,
        epochs=args.epochs,
        batch=args.batch,
        fliplr=0.0,        # A/B 표지, 좌우 차선 의미가 뒤집히면 안 됨
        mosaic=0.5,
        degrees=0.0,
        hsv_v=0.4,         # 조명 변화 대비
        close_mosaic=20,
        patience=40,
        name='tofu',
    )


if __name__ == '__main__':
    main()
