#!/usr/bin/env bash
# 모든 워크스페이스 빌드 (순서 = 의존 순서).  사용: ./build_all.sh [ws 이름 ...]
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
all=(sensor_ws yolo_ws lane_ws crosswalk_ws signal_ws slow_ws objavoid_ws cone_ws parking_ws mux_ws drive_ws bringup_ws)
if [[ $# -gt 0 ]]; then targets=("$@"); else targets=("${all[@]}"); fi
for ws in "${targets[@]}"; do
  echo "=== $ws"
  (cd "$root/$ws" && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release)
  source "$root/$ws/install/local_setup.bash"
done
echo '완료: source source_all_ws.bash'
