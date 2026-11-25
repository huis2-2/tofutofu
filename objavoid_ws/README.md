# objavoid_ws — 정적 장애물 (LIMO 7대) 차선 변경 회피 (미션 7)

`/yolo/fixed_obstacle` 확인 후, LiDAR 로 앞 차로(차 폭 통로, `check_dist` 안)가 막히면
비어 있고 **점선인 쪽** 옆 차로로 `/lane/change` 를 보냄 → lane_node 가 옆 차로를 추적, 조향은 Pure Pursuit 그대로.
갈 곳이 없으면 감속/정지 (접촉 -1점 방지). 장애물이 `exit_distance` 동안 안 보이면 종료.
`lidar_x`, `lidar_yaw` 는 rviz 에서 `/scan` 을 보고 앞 물체가 +x 로 오게 맞추세요.

## 빌드

```bash
cd ~/tofutofu/objavoid_ws && source env.sh
colcon build --symlink-install && source env.sh
```
