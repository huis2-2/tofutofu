# tofutofu — Autorace 2026 RC카 자율주행

> **처음이면 [docs/GUIDE.md](docs/GUIDE.md) 부터** — 젯슨 준비, 매일 하는 순서(pull → 주행 → 업로드 → 커밋 → 태그), 지금 할 일 순서가 한 장에 있습니다.

Jetson Orin Nano · ROS 2 Jazzy · RC카 (서보 + ESC + 엔코더) · USB 카메라 · RPLIDAR.
목표: **최대한 빠르게** 1바퀴 + 8개 미션. 구조는 [2026-AutoRace](https://github.com/Minjea31/2026-AutoRace)처럼
기능별 워크스페이스로 나누고 mux 가 우선순위로 최종 명령을 고릅니다. 토픽·우선순위 명세: [docs/architecture.md](docs/architecture.md)

| 구분 | 워크스페이스 | 담당 |
|---|---|---|
| 공통 | [sensor_ws](sensor_ws/) | 카메라(저지연), 엔코더(+속도 m/s·거리), LiDAR |
| 공통 | [yolo_ws](yolo_ws/) | YOLO11n-seg TensorRT 하나로 차선 마스크 + 표지판/신호등/콘/장애물 |
| 공통 | [lane_ws](lane_ws/) | 마스크 → 왜곡 보정·BEV → 중앙 경로 → Pure Pursuit (`/mission/lane`), 갈림길 A/B, 지그재그 직진 |
| 미션 1 | [signal_ws](signal_ws/) | 신호등 정지선 정지·초록 출발 |
| 미션 3 | [crosswalk_ws](crosswalk_ws/) | 횡단보도 정지 ±5cm, 3초 대기 |
| 미션 4 | [parking_ws](parking_ws/) | T자 전면 주차 (윗줄 가운데 칸) |
| 미션 5·6 | [slow_ws](slow_ws/) | 서행 표지·방지턱·마름모·지그재그 서행 |
| 미션 7 | [objavoid_ws](objavoid_ws/) | LiDAR 로 LIMO 감지 → 점선 쪽 차선 변경 |
| 미션 8 | [cone_ws](cone_ws/) | LiDAR 콘 사이 주행 (차선 무시) |
| 공통 | [mux_ws](mux_ws/) | 미션 우선순위 → `/drive/cmd` |
| 공통 | [drive_ws](drive_ws/) | PCA9685 서보/ESC + 엔코더 속도 PID |
| 실행 | [bringup_ws](bringup_ws/) | 전체 launch + rosbag 기록 |

## 빌드·실행 (Jetson)

```bash
sudo apt install ros-jazzy-ackermann-msgs ros-jazzy-rosbag2-storage-mcap python3-smbus2
pip install ultralytics Jetson.GPIO
cd ~/tofutofu && ./build_all.sh
source source_all_ws.bash
ros2 launch autorace_bringup autorace.launch.py     # 주행 + rosbag 기록
./tools/upload_bag.sh --all                         # 주행 후: 검사 + GitHub 업로드
```

## 처음 할 일 (실측값 → 설정 파일의 `TODO(실측)`)

| 값 | 어디에 | 어떻게 |
|---|---|---|
| ~~엔코더~~ 0.00412 m/pulse, 방향 = ESC 명령 | `sensors.yaml` | 반영 완료 |
| ~~최대 조향각~~ 좌 20° / 우 23°, ~~축거~~ 0.26 m | `drive.yaml`, `lane.yaml`, `parking.yaml`, `cone.yaml` | 반영 완료 |
| ~~ESC~~ 정지 1680 / 시작 1765 / 최고 1950 | `drive.yaml`, `drive_record.py` | 반영 완료. `kff` 는 `open_loop: true` 로 시험 |
| 카메라 렌즈·바닥 캘리브레이션 (어안) | `lane.yaml` `calib_file` | 아이폰 16 화면으로: [lane_ws/README.md](lane_ws/README.md) 캘리브레이션 절 |
| LiDAR 방향 (`lidar_yaw`) | `objavoid.yaml`, `cone.yaml` | rviz 로 `/scan` 확인 (위치는 앞바퀴 축 위로 반영) |
| 주차 칸 폭 | `parking.yaml` `slot_width` | 트랙에서 실측 |

## 만드는 순서
1. drive_node + 엔코더 속도 PID (`missions:=false`, 차를 띄워 놓고 먼저 시험)
2. yolo_node → lane_node → controller_node 로 트랙 주행
3. 최고속 튜닝 (`lane.yaml` 의 `v_max`, `a_lat_max`, lookahead)
4. 미션 (배점 큰 것부터: 신호등·서행 유도선·라바콘·주차)

## 데이터
- `drive_record.py`: 키보드/조이스틱 수동 주행 + 녹화 (이 스크립트가 카메라를 직접 열기 때문에 camera_node 와 동시 실행 X)
- `bag_to_images.py`, `bag_to_video.py`: rosbag → 라벨링 사진 / mp4
- [datasets/](datasets/): 2026-11-23 트랙 데이터
- [docs/training_guide.md](docs/training_guide.md): YOLO 재학습 가이드라인
- [docs/git_workflow.md](docs/git_workflow.md): **GitHub 사용 규칙** (main 하나, 잘 달리면 태그, rosbag 업로드)
- 주행은 기본으로 rosbag 기록 → 주행 후 `./tools/upload_bag.sh --all` (품질 검사 + Release 업로드 + [목록](datasets/bags_index/))
- `record:=true` 로 자율주행하면 `datasets/bags/` 에 이미지 + 명령 + PWM + 엔코더가 쌓임 (모방학습용)
