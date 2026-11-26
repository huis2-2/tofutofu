# GitHub 사용 규칙

## 기본: `main` 하나만 쓴다

- `main` = 지금 차에 올릴 코드. 노트북(Claude)도 젯슨도 `main` 에 커밋·푸시
- 젯슨에서 받기:
  ```bash
  cd ~/tofutofu && git pull && ./build_all.sh && source source_all_ws.bash
  ```
- 큰 구조 변경처럼 위험한 작업만 브랜치에서 하고, 젯슨에서 검증되면 `main` 에 합친다

## 규칙 5개

1. **pull 먼저, 그다음 작업**: 젯슨이든 노트북이든 시작할 때 `git pull`. 둘이 같은 파일(특히 yaml)을 동시에 고치면 충돌 난다
2. **젯슨에서 튜닝한 값은 바로 커밋**: `git add -A && git commit -m "튜닝: v_max 1.2→1.4 (랩 32초)" && git push`
   - 메시지에 무엇을 왜 바꿨는지, 결과(랩타임 등)를 쓴다 → 나중에 어떤 값이 좋았는지 찾을 수 있다
3. **잘 달리면 태그** (아래)
4. **git 에 올리지 않는 것**: `build/ install/ log/`, `*.engine`, rosbag 원본 (`.gitignore` 에 이미 있음)
   - rosbag 은 GitHub Release 로 (`./tools/upload_bag.sh`, 아래)
5. **브랜치를 바꾸거나 pull 하기 전에** `git status` 가 깨끗한지 확인

## 태그: "잘 달리던 버전" 표시

태그 = 특정 커밋에 붙이는 이름표. 코드가 바뀌어도 태그가 가리키는 커밋은 그대로라 언제든 돌아갈 수 있다.

**달기** (젯슨에서 잘 달린 직후, 커밋까지 끝낸 상태에서):
```bash
git tag -a good-1126-lap32 -m "랩 32초, 미션 6개 성공, v_max 1.4"
git push origin good-1126-lap32
```
이름 규칙: `good-<월일>-<특징>` (예: `good-1126-lap32`, `race-final`)

**어디서 보나**
- GitHub: 레포 → 오른쪽 위 브랜치 선택(`main ▾`) → **Tags** 탭, 또는 `https://github.com/huis2-2/tofutofu/tags`
- 터미널: `git tag -n` (태그와 메모 목록)

**그 버전으로 돌아가기**
```bash
git fetch --tags
git checkout good-1126-lap32          # 그 시점 코드로 바뀜 (읽기 전용 상태)
./build_all.sh && source source_all_ws.bash
# ... 주행 ...
git checkout main                      # 다시 최신으로
```
그 버전에서 계속 고쳐야 하면: `git checkout -b fix-from-good good-1126-lap32`

**대회 당일**: 마지막으로 잘 달린 태그로 체크아웃해 두고 그 상태로 출전 (전날 밤 실험 코드가 섞이지 않게)

## rosbag 기록 (모방학습 준비)

- `ros2 launch autorace_bringup autorace.launch.py` 는 **기본으로 기록** → `datasets/bags/run_YYYYmmdd_HHMMSS/`
- 주행이 끝나면:
  ```bash
  ./tools/upload_bag.sh --all
  ```
  1. `tools/check_bag.py` 로 품질 검사 → 각 기록 폴더에 `report.md` (✅ OK / ⚠️ CAUTION / ❌ BAD)
  2. GitHub Release `bags-YYYYMMDD` 에 zip 업로드 (파일 하나 2GB 까지)
  3. `datasets/bags_index/README.md` 목록 표 + 보고서를 `main` 에 커밋·푸시
- 처음 한 번 (젯슨): `sudo apt install gh zip && gh auth login`
- 하나만 검사: `python3 tools/check_bag.py datasets/bags/run_xxx`
- 젯슨 디스크가 차면 업로드 끝난 `datasets/bags/run_*` 는 지워도 된다 (Release 에 원본 있음)

판정 기준 (`check_bag.py`):

| 판정 | 조건 |
|---|---|
| ❌ BAD | 카메라나 조향 라벨 없음, 10초 미만, 카메라 15fps 미만, 움직인 시간 20% 미만, 2m 미만 |
| ⚠️ CAUTION | 카메라 25fps 미만, 0.5초 넘게 끊김, 라벨 없는 프레임 10% 이상, 직진뿐, 차선 인식 80% 미만 |
| ✅ OK | 위에 해당 없음 |
