# 노드 구조와 토픽 명세

구조는 [2026-AutoRace (충북대)](https://github.com/Minjea31/2026-AutoRace)를 따릅니다.
기능마다 워크스페이스 하나(`<기능>_ws/src/autorace_<기능>`), 패키지 안은 `core.py`(ROS와 무관한 로직, pytest로 시험) + `node.py`(ROS 연결) + `config/*.yaml` + `launch/`.
미션은 `/mission/<기능>/command_NN`으로 후보 명령을 내고, mux가 고정 우선순위로 하나를 골라 `/drive/cmd`로 보냅니다.

참고 레포와 다른 점 (차가 LIMO가 아니라 RC카라서):

- 차량 출력이 `/cmd_vel`(LIMO base)이 아니라 `drive_ws`의 `drive_node`(PCA9685 서보/ESC + 엔코더 속도 PID)
- 차선은 HSV/U-Net 대신 **YOLO-seg 하나**의 차선 마스크 → BEV → 중앙 경로 → Pure Pursuit (PID 대신)
- 정지 위치(±5cm)는 카메라 거리 + **엔코더 누적 거리**로 맞춤 (참고 레포는 흰색 ROI + 시간 지연)

## 흐름

```text
camera_node ─/camera/image_raw─► yolo_node ─/yolo/lane_mask─► lane_node ─/lane/path, /lane/target─► controller_node ─/mission/lane─┐
                                    │                            ▲  ▲                                                          │
                                    ├─/yolo/LR_sign──────────────┘  └─/lane/change── objavoid_node ◄─/scan── sllidar_node      │
                                    ├─/yolo/signal ─► signal_node ────────/mission/signal/command_02──────────────────────────┤
                                    ├─/yolo/slow_sign ─► slow_node ───────/mission/slow/command_04────────────────────────────┤
                                    ├─/yolo/parking_sign ─► parking_node ─/mission/parking/command_03─────────────────────────┤
                                    ├─/yolo/cone ─► cone_node ◄─/scan ────/mission/cone/command_06────────────────────────────┤
                                    └─/yolo/fixed_obstacle ─► objavoid_node /mission/objavoid/command_05──────────────────────┤
                     lane_node /lane/target (횡단보도 거리) ─► crosswalk_node /mission/crosswalk/command_01 ────────────────────┤
                                                                                                                              ▼
encoder_node ─/encoder/speed, /encoder/distance─► (controller, 미션들, drive)                        mux_node ─/drive/cmd─► drive_node ─► PCA9685
```

## 미션 우선순위 (mux)

| 우선 | 워크스페이스 | 토픽 | 조향 |
|---|---|---|---|
| 01 | crosswalk_ws | `/mission/crosswalk/command_01` | lane 그대로, 속도만 (정지) |
| 02 | signal_ws | `/mission/signal/command_02` | lane 그대로, 속도만 (정지) |
| 03 | parking_ws | `/mission/parking/command_03` | 접근은 lane, 진입은 자체 |
| 04 | slow_ws | `/mission/slow/command_04` | lane 그대로, 속도만 |
| 05 | objavoid_ws | `/mission/objavoid/command_05` | lane 그대로 (차선 변경은 `/lane/change`) |
| 06 | cone_ws | `/mission/cone/command_06` | 자체 (콘 사이) |
| 기본 | lane_ws | `/mission/lane` | Pure Pursuit |

- 모든 명령 타입: `ackermann_msgs/AckermannDriveStamped` (speed m/s, steering_angle rad, + = 왼쪽)
- 미션은 **활성일 때만 발행**. mux는 0.25초 안에 들어온 명령 중 우선순위가 가장 높은 것을 선택, 정지 중인 미션은 속도 0을 계속 발행해서 제어권 유지
- 아무 명령이 없으면 mux가 속도 0, drive_node는 0.3초간 명령이 없으면 중립

## 토픽

| 토픽 | 타입 | 발행 | 내용 |
|---|---|---|---|
| `/camera/image_raw` | Image bgr8 | camera_node | 640x480 MJPG, BEST_EFFORT depth 1 |
| `/camera/image_raw/compressed` | CompressedImage | camera_node | `record:=true` 일 때 (rosbag용) |
| `/encoder/speed` | Float32 | encoder_node | m/s, 50 Hz |
| `/encoder/distance` | Float64 | encoder_node | 누적 거리 m (미션 거리 기준) |
| `/scan` | LaserScan | sllidar_node | |
| `/yolo/lane_mask` | Image mono8 320x240 | yolo_node | 1 yellow, 2 dashed, 3 horizontal, 4 zigzag, 5 cross_walk, 6 parking_line, 7 diamond |
| `/yolo/detections` | Float32MultiArray | yolo_node | `[class_idx, conf, x1, y1, x2, y2] * N` |
| `/yolo/slow_sign` `/yolo/parking_sign` `/yolo/no_speed_limit` `/yolo/cone` `/yolo/fixed_obstacle` | Bool | yolo_node | 매 프레임 (연속 2프레임 확인) |
| `/yolo/LR_sign` | Int8 | yolo_node | +1 A(왼쪽) / -1 B(오른쪽) |
| `/yolo/signal` | Int8 | yolo_node | 0 빨강 / 1 노랑 / 2 초록 |
| `/lane/path` | nav_msgs/Path | lane_node | 중앙 경로 (base_link = 뒷바퀴 축 원점) |
| `/lane/target` | Float32MultiArray | lane_node | 아래 표 |
| `/lane/zigzag` | UInt8 | lane_node | 0 / 1 한쪽 / 2 양쪽 |
| `/lane/change` | Int8 | objavoid_node | +1 왼쪽 / -1 오른쪽 차선으로 |
| `/drive/cmd` | AckermannDriveStamped | mux_node | 최종 명령 |
| `/mux/active_source` | String | mux_node | 지금 제어 중인 미션 |
| `/drive/steering_us` `/drive/throttle_us` | Int32 | drive_node | 실제 PWM (drive_record.py 와 같은 토픽) |
| `/drive/steering` | Float32 | drive_node | -1 오른쪽 ~ +1 왼쪽 |
| `/drive/estop` | Bool | (사용자) | true = 즉시 정지 |
| `/mux/enable` | Bool | (사용자) | false = 정지 |

### `/lane/target` 인덱스 (`lane_core.T_*`)

| idx | 이름 | 단위 |
|---|---|---|
| 0 | valid | 1/0 |
| 1 | lateral_error (x_ref 에서 중앙선 y, + = 중앙이 왼쪽) | m |
| 2 | heading | rad |
| 3 | curvature | 1/m |
| 4 | stop_line: 앞바퀴 → 정지선(horizontal_line) | m, 없으면 -1 |
| 5 | crosswalk: 앞바퀴 → 횡단보도 | m, -1 |
| 6 | parking: 앞바퀴 → 오른쪽 주차선 | m, -1 |
| 7 | diamond: 앞바퀴 → 서행 마름모 | m, -1 |
| 8, 9 | left / right 경계 찾음 | 1/0 |
| 10, 11 | left / right 경계의 점선 비율 | 0~1 |
| 12 | zigzag | 0/1/2 |
| 13 | fork bias | +1 / -1 / 0 |
| 14 | 추정 차선 폭 | m |

## 좌표계

`base_link`: 원점 = 뒷바퀴 축 중앙 바닥, x 앞, y 왼쪽. 미션 판정 기준(앞바퀴)은 원점 + wheelbase.
