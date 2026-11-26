# bringup_ws — 전체 실행 + rosbag 기록

```bash
ros2 launch autorace_bringup autorace.launch.py                 # 주행 + rosbag 기록 (기본, datasets/bags/run_*)
ros2 launch autorace_bringup autorace.launch.py record:=false   # 기록 없이
ros2 launch autorace_bringup autorace.launch.py drive:=false debug:=true   # 바퀴 출력 없이 인식 확인
ros2 launch autorace_bringup autorace.launch.py missions:=false # 차선 주행만 (최고속 튜닝)
```
센서 설정: `config/sensors.yaml`. 나머지는 각 패키지의 `config/*.yaml`.
기본으로 카메라 JPEG, 명령/PWM, 엔코더, LiDAR, 인식 결과를 mcap 으로 기록합니다 (1분 약 70MB, 모방학습용).
주행 후 `./tools/upload_bag.sh --all` → 품질 검사 + GitHub Release 업로드 + `datasets/bags_index/` 목록 커밋.

## 빌드

```bash
cd ~/tofutofu/bringup_ws && source env.sh
colcon build --symlink-install && source env.sh
```
