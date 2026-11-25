# yolo_ws — YOLO-seg 하나로 차선 마스크 + 표지판/신호등

`autorace_yolo/yolo_node` 가 `/camera/image_raw` 를 받아 TensorRT 엔진으로 추론하고,
차선 계열은 **클래스 값 마스크**(`/yolo/lane_mask`, mono8)로, 표지판·신호등·장애물은 **박스/Bool** 로 발행합니다.

## 출력

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/yolo/lane_mask` | Image mono8 320x240 | 1 yellow_line, 2 dashed_line, 3 horizontal_line, 4 zigzag_line, 5 cross_walk, 6 parking_line, 7 diamond |
| `/yolo/detections` | Float32MultiArray | `[class_idx, conf, x1, y1, x2, y2] * N` (정규화, idx = `classes.SIGN_NAMES`) |
| `/yolo/slow_sign`, `/yolo/parking_sign`, `/yolo/no_speed_limit`, `/yolo/cone`, `/yolo/fixed_obstacle` | Bool | 매 프레임, 연속 2프레임 확인 |
| `/yolo/LR_sign` | Int8 | +1 = A(왼쪽), -1 = B(오른쪽). 확인됐을 때만 |
| `/yolo/signal` | Int8 | 0 빨강, 1 노랑, 2 초록. 확인됐을 때만 (빨강 우선) |

클래스 이름은 `autorace_yolo/classes.py` 에서 정규화합니다 (`Go_A` → `go_a`, `Dashed_line` → `dashed_line`).

## 학습 (Colab)

`train/train_colab.py` — yolo11n-seg, imgsz 640, **fliplr=0**.
라벨링 주의: 모든 이미지에서 보이는 차선은 전부 라벨링, `datasets/` 자체 카메라 사진 포함.

## Jetson 에서 엔진 변환

```bash
pip install ultralytics   # JetPack 의 torch / tensorrt 사용
cp best.pt ~/tofutofu/yolo_ws/model/
~/tofutofu/yolo_ws/train/export_engine.sh
```

## 빌드·실행

```bash
cd ~/tofutofu/yolo_ws && source env.sh
colcon build --symlink-install && source env.sh
ros2 launch autorace_yolo yolo.launch.py
```

추론 시간은 노드 로그에 150프레임마다 출력됩니다. 확인용 영상은 `publish_debug:=true` 후 `rqt_image_view /yolo/debug/compressed`.
