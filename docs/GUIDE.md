# 작업 가이드 (이것만 보면 됨)

젯슨에서는 **`main` 만** 씁니다. 명령은 전부 `~/tofutofu` 에서 실행.

---

## A. 처음 한 번만 (젯슨 준비)

```bash
sudo apt install ros-jazzy-ackermann-msgs ros-jazzy-rosbag2-storage-mcap python3-smbus2 gh zip
pip install ultralytics Jetson.GPIO
gh auth login                      # GitHub 로그인 (rosbag 업로드용)
cd ~ && git clone https://github.com/huis2-2/tofutofu.git
cd ~/tofutofu && ./build_all.sh
```

---

## B. 매일 하는 순서

### 1) 시작: 최신 코드 받기
```bash
cd ~/tofutofu
git status                         # 바뀐 파일 없는지 확인 (있으면 아래 4번처럼 커밋 먼저)
git pull
./build_all.sh && source source_all_ws.bash
```

### 2) 주행 (자동으로 rosbag 기록됨)
```bash
ros2 launch autorace_bringup autorace.launch.py                       # 주행 + 기록
ros2 launch autorace_bringup autorace.launch.py drive:=false debug:=true   # 바퀴 출력 없이 인식만 확인
ros2 launch autorace_bringup autorace.launch.py missions:=false       # 차선 주행만 (속도 튜닝)
```
화면 확인 (다른 터미널): `rqt_image_view /yolo/debug/compressed`, `rqt_image_view /lane/debug`

### 3) 주행 끝: 기록 검사 + 업로드
```bash
./tools/upload_bag.sh --all
```
- 기록마다 ✅ OK / ⚠️ CAUTION / ❌ BAD 판정 → GitHub Release 에 업로드 → 목록 커밋까지 자동
- 목록: [datasets/bags_index/](../datasets/bags_index/) · 원본: GitHub → Releases
- 업로드 끝난 `datasets/bags/run_*` 는 지워도 됨

### 4) 값을 바꿨으면: 바로 커밋
```bash
git add -A
git commit -m "튜닝: v_max 1.2→1.4 (랩 32초)"      # 무엇을 / 왜 / 결과
git push
```

### 5) 잘 달렸으면: 태그
```bash
git tag -a good-1126-lap32 -m "랩 32초, 미션 6개 성공"
git push origin good-1126-lap32
```
- 보는 곳: https://github.com/huis2-2/tofutofu/tags
- 그 버전으로 돌아가기: `git checkout good-1126-lap32 && ./build_all.sh` (최신으로: `git checkout main`)
- **대회 당일**: 마지막으로 잘 달린 태그로 체크아웃해 두고 출전

---

## C. 큰 작업 순서 (지금 할 일)

| 순서 | 할 일 | 자세히 |
|---|---|---|
| 1 | 카메라 각도·위치 **최종 고정** | — |
| 2 | 아이폰 캘리브레이션 (렌즈 왜곡 → 바닥 매핑) | [lane_ws/README.md](../lane_ws/README.md) 캘리브레이션 절, 패턴: `lane_ws/tools/phone_patterns/` |
| 3 | 지금 모델(best.pt) 엔진 변환 → `drive:=false debug:=true` 로 인식 확인 + 기록 | [training_guide.md](training_guide.md) ② |
| 4 | 안 되는 장면 + 부족한 클래스 데이터 추가, 라벨링 | [training_guide.md](training_guide.md) ③ |
| 5 | Roboflow 새 version (resize X, 좌우 뒤집기 X) → Colab 재학습 (`fliplr=0`) | [training_guide.md](training_guide.md) ④ |
| 6 | drive_node 속도 PID 튜닝 (`open_loop: true` 로 `kff` 먼저) | [drive_ws/README.md](../drive_ws/README.md) |
| 7 | 차선 주행 → 최고속 튜닝 (`lane.yaml` `v_max`, `a_lat_max`) | [lane_ws/README.md](../lane_ws/README.md) |
| 8 | 미션 붙이기 (배점 큰 것부터: 신호등·서행 유도선·라바콘·주차) | 각 `*_ws/README.md` |

---

## D. 헷갈릴 때

| 상황 | 할 일 |
|---|---|
| `git pull` 이 충돌난다 / 안 된다 | `git status` 로 바뀐 파일 확인 → 커밋하거나 `git checkout -- <파일>` 로 되돌린 뒤 다시 pull |
| 빌드가 이상하다 | `rm -rf */build */install */log && ./build_all.sh` (빌드 결과는 지워도 다시 생김) |
| 예전에 잘 달리던 상태로 | 위 5) 태그로 돌아가기 |
| 차가 안 멈춘다 | `ros2 topic pub --once /drive/estop std_msgs/Bool "{data: true}"` 또는 RC 조종기로 |
| 어떤 미션이 제어 중인지 | `ros2 topic echo /mux/active_source` |
| 미션 하나만 끄고 시험 | `mux_ws/src/autorace_mux/config/mux.yaml` 의 `disabled_sources: ['cone']` |

더 자세한 규칙: [git_workflow.md](git_workflow.md) · 노드·토픽 구조: [architecture.md](architecture.md)
