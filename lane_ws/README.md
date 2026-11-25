# lane_ws — 차선 인식 + 차선 추종 제어 + 갈림길 A/B

## lane_node
`/yolo/lane_mask` → (렌즈 왜곡 보정 + BEV 를 **픽셀별 지면좌표 LUT** 로 한 번에) → 지면 좌표 점 →
좌/우 경계 sliding-window 추적 → 2차 다항식 → 중앙 경로 `/lane/path`, 요약 `/lane/target`.

- 경계선 = yellow_line + dashed_line + zigzag_line
- 한쪽만 보이면 그 경계에서 **법선 방향**으로 차선폭/2 (커브에서도 정확)
- 지그재그 쪽은 직선 피팅 → 지그재그를 따라 흔들리지 않고 직진 헤딩 유지 (미션 6)
- 갈림길: `/yolo/LR_sign` 을 받으면 `fork_hold_distance` 동안 A=왼쪽 경계만 / B=오른쪽 경계만 추종 (미션 2)
- `/lane/change` (+1/-1) 를 받으면 추적 차선을 옆으로 옮김 (objavoid 미션 7)
- 정지선/횡단보도/주차선/마름모까지 **앞바퀴 기준 거리**를 계산해 미션 노드에 제공

## controller_node
Pure Pursuit, lookahead `Ld = lookahead_base + lookahead_gain * v` (속도 비례).
속도 = `min(v_max, sqrt(a_lat_max / κ_max))` (경로 전체 최대 곡률) × 헤딩 오차 감속. `/yolo/no_speed_limit` 이면 `v_boost`.
가속만 제한, 감속은 즉시. 차선을 놓치면 `lost_hold_sec` 동안 직전 조향으로 서행 후 정지.

## 캘리브레이션 (속도에 중요)
처음엔 `lane.yaml` 의 `cam_h`, `pitch_deg`, `hfov_deg` 핀홀 근사로 동작. 정확도를 올리려면:

```bash
cd ~/tofutofu/lane_ws/tools
python3 calibrate_camera.py capture --out calib_imgs          # 체커보드 20장+
python3 calibrate_camera.py solve calib_imgs --cols 9 --rows 6 --square 0.025 \
    --out ../src/autorace_lane/config/camera_calib.yaml        # 광각이 심하면 --fisheye
python3 calibrate_ground.py --calib ../src/autorace_lane/config/camera_calib.yaml \
    --cols 9 --rows 6 --square 0.025 --origin-x 0.45 --origin-y -0.10
```
그 다음 `lane.yaml` 의 `calib_file` 에 yaml 경로를 넣습니다. 확인: `publish_debug: true` 후 `rqt_image_view /lane/debug` (위=앞, 1px=1cm).

BEV 앞쪽 범위(`x_max`)를 늘릴수록 lookahead 를 길게 잡을 수 있어 고속에 유리합니다 → 카메라를 높이고 덜 숙이는 것이 좋습니다.

## 빌드

```bash
cd ~/tofutofu/lane_ws && source env.sh
colcon build --symlink-install && source env.sh
```
