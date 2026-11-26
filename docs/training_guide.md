# YOLO-seg 학습 가이드라인

순서: **① 카메라 고정·캘리브레이션 → ② 지금 모델로 실차 인식 확인(+rosbag) → ③ 데이터 확충 → ④ 재학습 → ⑤ 비교·배포**

## 지금 모델 (2026-11-26, `tofu_v3` version 4)

| 항목 | 값 |
|---|---|
| 모델 | yolo11n-seg, imgsz 640, 94 epoch (patience 30), AdamW |
| 성능 | box mAP50 0.92 / mask mAP50 0.91, mask mAP50-95 0.63 |
| 문제 1 | **`fliplr: 0.5`** (좌우 뒤집기) → Go_A/Go_B 혼동 가능. 재학습 때 반드시 0 |
| 문제 2 | 라벨 0개인 옛 클래스 `A`, `B`, `slow_sign`, `tofu-v3` 가 남아 있음 (코드는 이름 정규화로 동작) |
| 문제 3 | val seg_loss 가 20 epoch 부근부터 상승 → 과적합. 데이터 확충으로 해결 |

## ① 카메라 고정 + 캘리브레이션

**카메라 각도·위치를 먼저 최종 확정.** 이후 바꾸면 캘리브레이션과 데이터 수집을 다시 해야 한다.
아이폰 캘리브레이션: [lane_ws/README.md](../lane_ws/README.md) 의 캘리브레이션 절. 패턴 이미지는 `lane_ws/tools/phone_patterns/`.

## ② 지금 모델로 인식 확인

```bash
cp best.pt ~/tofutofu/yolo_ws/model/best.pt && ~/tofutofu/yolo_ws/train/export_engine.sh
ros2 launch autorace_bringup autorace.launch.py drive:=false debug:=true record:=true
rqt_image_view /yolo/debug/compressed      # 다른 터미널
rqt_image_view /lane/debug
```
차는 RC 조종기나 손으로 한 바퀴. 안 되는 장면은 시간을 메모 (③에서 데이터로 사용).

| 항목 | 기준 |
|---|---|
| 노란선 / 점선 | BEV 에서 끊기지 않고 선 모양, 커브에서도 |
| 지그재그 | `zigzag_line` 이 노란선과 구분 |
| 횡단보도 / 정지선 | 가까이 다가가도 계속 잡힘 |
| Go_A / Go_B | `ros2 topic echo /yolo/LR_sign` → A = +1, B = -1 (뒤집기 학습 때문에 제일 의심) |
| 신호등 | 빨강·노랑·초록 각각, 방해 모니터를 신호등으로 착각하지 않음 |
| 표지판 | 서행·P·콘·장애물을 미션 전에 충분히 먼 거리에서 |
| 속도 | 노드 로그 `YOLO 평균 ms` ≤ 33 ms (30 fps) |

## ③ 데이터 확충

```bash
python3 bag_to_images.py datasets/bags/run_XXXX --fps 3     # 초당 2~3장이면 충분 (옆 프레임은 거의 같음)
```

| 클래스 | 지금 | 목표 | 메모 |
|---|---|---|---|
| `Go_A` / `Go_B` | 54 / 39 | 각 100+ | 먼 거리, 가까운 거리, 비스듬한 각도 |
| `red/yellow/green_light` | 55 / 28 / 30 | 각 100+ | 방해 모니터가 같이 보이는 장면 많이 |
| `no_speed_limit` | 9 | 50+ | |
| `fixed_obstacle` | 91 | 150+ | 3차선에 LIMO 여러 대 |
| 차선류 | 충분 | — | 커브·3차선·갈림길·주차 구역 위주 추가 |

라벨링 규칙 (개수보다 일관성):
- 보이는 차선은 화면 끝에 잘린 것까지 **전부**
- 선은 **테두리를 따라 얇게** (주변 바닥까지 뭉툭하게 칠하지 않기)
- 점선은 **조각마다 따로**, 처음부터 끝까지 같은 방식
- 방해 모니터 속 빨강·초록 그래픽은 **라벨링하지 않음** (신호등 아님을 배우는 데이터)
- 클래스 정리: `A`, `B`, `slow_sign`, `tofu-v3` 삭제. `slow`/`slow_down` 은 의미가 같으면 합치기
- valid/test 는 학습과 **다른 주행(세션)** 에서 고르기 (옆 프레임이 섞이면 점수가 부풀려짐)

## ④ 재학습

Roboflow version:
- 전처리: auto-orient 만. **resize 넣지 않기** (640x640 늘리기 = 실제 640x480 과 비율이 달라짐)
- 증강: **좌우 뒤집기 없이** (밝기·노출·shear 정도)

Colab: `python train_colab.py --version <N>` ([yolo_ws/train/train_colab.py](../yolo_ws/train/train_colab.py))

| 설정 | 값 | 이유 |
|---|---|---|
| `fliplr` | **0** | A/B·좌우 의미 보존 |
| `imgsz` | 640 | |
| `epochs` / `patience` | 150 / 40 | |
| `mosaic` / `close_mosaic` | 0.5 / 20 | 마지막 20 epoch 은 실제 사진처럼 |
| 모델 | yolo11n-seg | ② 에서 ms 여유 있으면 `--model yolo11s-seg.pt` 도 시험 |

## ⑤ 비교·배포

- 같은 test 세트로 옛 best.pt 와 새 모델 비교. 전체 mAP 보다 **Go_A/Go_B 혼동표**, **신호등 클래스별 점수**
- Jetson 에서 엔진 변환 → ② 확인 항목 다시
