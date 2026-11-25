# signal_ws — 신호등 정지·출발 (미션 1)

`/yolo/signal` 빨강/노랑 → 정지선(정지선 또는 횡단보도 거리)까지 엔코더 기반 감속 정지 → 초록 `green_confirm` 회 연속 → 출발.
다가가는 중 초록이 되면 정지하지 않고 통과. 정지선을 `blind_stop_sec` 동안 못 보면 그 자리 정지.
방해 모니터 오검출 대비: YOLO 연속 2프레임 확인 + 초록 연속 확인. 빨강이 초록보다 우선.

## 빌드

```bash
cd ~/tofutofu/signal_ws && source env.sh
colcon build --symlink-install && source env.sh
```
