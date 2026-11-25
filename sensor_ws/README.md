# sensor_ws — 센서 (카메라·엔코더·LiDAR)

| 패키지 | 노드 | 출력 |
|---|---|---|
| camera_node | camera_node | `/camera/image_raw` (MJPG 640x480, 캡처 스레드로 최신 프레임만, BEST_EFFORT depth 1, imshow 없음), 옵션 `/camera/image_raw/compressed` |
| encoder_node (C++) | encoder_node | `/encoder/count`, `/encoder/pulses`, `/encoder/direction`, **`/encoder/speed` (m/s)**, **`/encoder/distance` (m)** |
| sllidar_ros2 | sllidar_node | `/scan` |

엔코더 보정: 차를 손으로 정확히 1m 밀고 `/encoder/count` 변화량 N을 재서 `meters_per_count: 1/N`
(`bringup_ws/src/autorace_bringup/config/sensors.yaml`).

## 빌드

```bash
cd ~/tofutofu/sensor_ws && source env.sh
colcon build --symlink-install && source env.sh
```
