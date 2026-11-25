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

## 캘리브레이션 (아이폰 16 사용)

카메라가 어안이라 `lane.yaml` 의 높이·각도 근사만으로는 거리가 부정확합니다. 두 단계로 맞춥니다 (camera_node 끄고).

**0. 패턴 만들기** → 아이폰으로 AirDrop, 사진 앱에서 열고 화면 탭(UI 숨김), 밝기 최대, 자동 잠금 끔
```bash
cd ~/tofutofu/lane_ws/tools
python3 make_phone_pattern.py        # phone_checker.png (내부 코너 5x14, 한 칸 8.28mm), phone_marker.png (흰 화면)
```

**1. 렌즈 왜곡** — 폰에 `phone_checker.png`, 카메라에서 10~25cm, 가운데·귀퉁이·가장자리·기울여서 25장 이상
```bash
python3 calibrate_camera.py capture --cols 5 --rows 14 --out calib_imgs     # 초록 FOUND 일 때 스페이스
python3 calibrate_camera.py solve calib_imgs --cols 5 --rows 14 --fisheye --preview \
    --out ../src/autorace_lane/config/camera_calib.yaml                      # RMS 0.5px 이하면 좋음
```

**2. 바닥 매핑** — 폰에 `phone_marker.png`(흰 화면)를 띄워 **움직이는 점 마커**로 사용.
차를 직선 구간에 세우고 바닥에 테이프로 9~12점 표시 (앞바퀴 축 중심 기준, 예: x = 10/25/40/60cm × y = -25/0/+25cm)
```bash
python3 calibrate_ground_phone.py --calib ../src/autorace_lane/config/camera_calib.yaml
#  b: 배경(폰 없이) → 폰을 표시점 위에 놓고 c → 터미널에 "x y"(cm) 입력 → 반복 → f: 계산·저장 → v: 확인
```
`f` 후 점별 오차가 나옵니다 (2cm 넘는 점은 `u` 로 빼고 다시). `v` 를 누르면 예상 차선(노랑)과 앞바퀴 기준 거리 눈금이
화면에 겹쳐 보입니다 → 실제 차선과 겹치면 성공. 마지막으로 `lane.yaml` 의 `calib_file` 에 yaml 경로를 넣습니다.
(종이 체커보드를 바닥에 까는 `calibrate_ground.py` 도 그대로 있습니다.)

합성 테스트 (`tools/test_phone_tools.py`): 실측 자세(높이 11.5cm, 57°)의 어안 카메라에서 바닥 매핑 오차 1cm 이내.

BEV 앞쪽 범위(`x_max`)를 늘릴수록 lookahead 를 길게 잡을 수 있어 고속에 유리합니다 → 카메라를 높이고 덜 숙이는 것이 좋습니다.

## 빌드

```bash
cd ~/tofutofu/lane_ws && source env.sh
colcon build --symlink-install && source env.sh
```
