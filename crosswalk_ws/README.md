# crosswalk_ws — 보행자 표지 / 횡단보도 정지 (미션 3)

`/lane/target` 의 횡단보도 거리(앞바퀴 기준)가 `trigger_dist` 안에 들어오면 시작.
정지 목표를 **엔코더 누적 거리**로 고정하고 `v = sqrt(2·decel·남은거리)` 로 감속 → 앞바퀴를 횡단보도 앞 `stop_before` 에 정지.
완전 정지(속도 < 0.02) 확인 시점부터 `hold_sec`(3.5초) 대기 후 출발. 신호등이 최근에 보였으면(신호등 횡단보도) 무시.

상태: WAIT → APPROACH → HOLD → PASS → DONE. `/crosswalk/state` 로 확인.
튜닝: 실제 정지 위치를 재서 `stop_before` 조정, ESC 브레이크에 맞게 `decel`.

## 빌드

```bash
cd ~/tofutofu/crosswalk_ws && source env.sh
colcon build --symlink-install && source env.sh
```
