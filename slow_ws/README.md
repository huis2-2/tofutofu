# slow_ws — 서행 (방지턱 5, 서행 유도선 6)

트리거: `/yolo/slow_sign`, 노면 마름모(`/lane/target` diamond 거리), 지그재그(`/lane/zigzag`).
마지막 트리거 + `hold_distance` (엔코더 거리) 동안 `min(lane 속도 × ratio, max_speed)`. lane 조향 그대로.
대회 기준은 "육안으로 느려진 게 보이면" 이므로 비율(기본 45%)로 감속합니다.

## 빌드

```bash
cd ~/tofutofu/slow_ws && source env.sh
colcon build --symlink-install && source env.sh
```
