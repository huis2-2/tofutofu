#!/usr/bin/env bash
# 주행 rosbag 검사 → GitHub Release 에 업로드 → 보고서 목록을 레포에 커밋.
#
#   ./tools/upload_bag.sh datasets/bags/run_20261126_143000 [더 많은 bag 폴더 ...]
#   ./tools/upload_bag.sh --all          # datasets/bags/ 안에서 아직 안 올린 것 전부
#
# - rosbag 은 커서(1분 ~70MB) git 에 직접 못 넣음 (파일 100MB 제한) → Release 'bags-YYYYMMDD' 에 zip 으로
# - 레포에는 datasets/bags_index/<run>.md (보고서) 와 README.md (목록 표) 만 커밋
# - 판정이 BAD 여도 기록은 올림 (목록에 ❌ 표시). 학습할 때 목록 보고 고르면 됨
#
# 처음 한 번 (Jetson): sudo apt install gh zip && gh auth login
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
index_dir="datasets/bags_index"
index="$index_dir/README.md"
mkdir -p "$index_dir"
command -v gh >/dev/null || { echo 'gh 가 없습니다: sudo apt install gh && gh auth login' >&2; exit 1; }
repo="$(gh repo view --json nameWithOwner -q .nameWithOwner)"

if [[ "${1:-}" == "--all" ]]; then
  mapfile -t runs < <(find datasets/bags -mindepth 1 -maxdepth 1 -type d | sort)
else
  runs=("$@")
fi
[[ ${#runs[@]} -gt 0 ]] || { echo '사용법: upload_bag.sh <bag 폴더...> | --all' >&2; exit 1; }

if [[ ! -f "$index" ]]; then
  cat > "$index" <<'EOF'
# 주행 기록 (rosbag) 목록

rosbag 원본은 GitHub Releases (`bags-YYYYMMDD`) 에 zip 으로 있습니다. 판정 기준: `tools/check_bag.py`
✅ OK = 바로 학습, ⚠️ CAUTION = 보고서 확인 후 사용, ❌ BAD = 학습에 쓰지 않음

| 기록 | 판정 | 길이 (s) | 거리 (m) | 평균 속도 | fps | 제어 | 보고서 | 다운로드 |
|---|---|---|---|---|---|---|---|---|
EOF
fi

for run in "${runs[@]}"; do
  run="${run%/}"
  name="$(basename "$run")"
  [[ "$name" == rosbag ]] && name="$(basename "$(dirname "$run")")"
  if grep -q "| $name |" "$index"; then
    echo "이미 올림: $name (건너뜀)"
    continue
  fi
  echo "=== $name: 검사"
  python3 tools/check_bag.py "$run" > /dev/null || true
  run_dir="$run"
  [[ -f "$run/report.json" ]] || run_dir="$(dirname "$run")"
  rep="$run_dir/report.json"
  [[ -f "$rep" ]] || { echo "보고서 생성 실패: $run" >&2; continue; }
  cp "$run_dir/report.md" "$index_dir/$name.md"

  day="$(echo "$name" | grep -oE '20[0-9]{6}' | head -1)"
  day="${day:-$(date +%Y%m%d)}"
  tag="bags-$day"
  zip_path="/tmp/$name.zip"
  echo "=== $name: 압축"
  (cd "$(dirname "$run_dir")" && rm -f "$zip_path" && zip -q -r -0 "$zip_path" "$(basename "$run_dir")")
  size=$(stat -c %s "$zip_path")
  if (( size > 2000000000 )); then
    echo "2GB 넘음 ($size bytes): Release 제한. 나눠서 녹화하세요" >&2
    continue
  fi
  gh release view "$tag" >/dev/null 2>&1 || \
    gh release create "$tag" --title "주행 기록 $day" --notes "rosbag (mcap). 목록: datasets/bags_index/README.md"
  echo "=== $name: 업로드 ($((size / 1000000)) MB)"
  gh release upload "$tag" "$zip_path" --clobber
  rm -f "$zip_path"

  row="$(python3 - "$rep" "$name" "$tag" "$repo" <<'PY'
import json, sys
r = json.load(open(sys.argv[1])); name, tag, repo = sys.argv[2:5]
icon = {'OK': '✅', 'CAUTION': '⚠️', 'BAD': '❌'}[r['verdict']]
g = lambda k: r.get(k, '-')
src = ','.join(r.get('sources', [])) or '-'
print(f"| {name} | {icon} {r['verdict']} | {g('duration_sec')} | {g('distance_m')} | {g('speed_mean_moving')} | "
      f"{g('camera_fps')} | {src} | [보고서]({name}.md) | "
      f"[zip](https://github.com/{repo}/releases/download/{tag}/{name}.zip) |")
PY
)"
  echo "$row" >> "$index"
  echo "$row"
done

git add "$index_dir"
if ! git diff --cached --quiet; then
  git commit -q -m "Add driving bag reports: $(date +%Y-%m-%d)"
  git push -q
  echo '목록 커밋·푸시 완료'
fi
