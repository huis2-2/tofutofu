# cone_ws — 라바콘 회피 (미션 8)

`/yolo/cone` 확인 후 LiDAR 콘 군집이 `min_cones` 개 이상이면 시작. 차선 무시.
각도순 인접 거리로 군집 → 크기 `max_cone_size` 이하만 콘 → 가장 가까운 왼쪽/오른쪽 콘의 중점(한쪽만 보이면 `half_gap` 안쪽)을 목표로 Pure Pursuit.
콘이 `exit_distance` 동안 안 보이면 종료 → lane 복귀.

## 빌드

```bash
cd ~/tofutofu/cone_ws && source env.sh
colcon build --symlink-install && source env.sh
```
