# parking_ws — T자 주차 (미션 4, 전면 주차 / 윗줄 가운데 칸)

`/yolo/parking_sign` → ARMED (lane 따라 `approach_speed`) → 오른쪽 주차선 첫 가장자리 위치를 엔코더 거리로 고정 →
뒷바퀴가 `칸 중심 - turn_radius + turn_offset` 에 오면 오른쪽 최대 조향으로 90도 원호 → `enter_distance` 직진 → 정지 유지.

실측 필요: `turn_radius` (최대 조향으로 원 그려 지름/2), `slot_width`. 시험하면서 `turn_offset`, `turn_scale`, `enter_distance` 조정.

## 빌드

```bash
cd ~/tofutofu/parking_ws && source env.sh
colcon build --symlink-install && source env.sh
```
