# sensor_ws — 센서 (카메라·엔코더·LiDAR)

| 패키지 | 노드 | 출력 |
|---|---|---|
| camera_node | camera_node | `/camera/image_raw` (MJPG 640x480, 캡처 스레드로 최신 프레임만, BEST_EFFORT depth 1, imshow 없음), 옵션 `/camera/image_raw/compressed` |
| encoder_node (C++) | encoder_node | `/encoder/count`, `/encoder/pulses`, `/encoder/direction`, **`/encoder/speed` (m/s)**, **`/encoder/distance` (m)** |
| sllidar_ros2 | sllidar_node | `/scan` |

엔코더 거리·속도 = Total Pulses(`/encoder/pulses`) 변화량 × **0.00412 m** (`meters_per_pulse`).
방향은 엔코더 A/B 가 아니라 **ESC 명령** `/drive/throttle_us` 로 판단: 1680 보다 크면 전진, 작으면 후진.
주행 중 1680 아래 펄스는 브레이크이므로, 바퀴가 `switch_stop_time`(0.1초) 이상 멈춰 있을 때만 방향을 바꿉니다.
`/drive/throttle_us` 는 drive_node 와 drive_record.py 가 발행합니다. 설정: `bringup_ws/src/autorace_bringup/config/sensors.yaml`.

## 빌드

```bash
cd ~/tofutofu/sensor_ws && source env.sh
colcon build --symlink-install && source env.sh
```
