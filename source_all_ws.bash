#!/usr/bin/env bash
# 모든 워크스페이스를 현재 터미널에 로드.  사용: source ~/tofutofu/source_all_ws.bash
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo '실행 방법: source source_all_ws.bash' >&2
  exit 1
fi
_tofu_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
for _ws in sensor_ws yolo_ws lane_ws crosswalk_ws signal_ws slow_ws objavoid_ws cone_ws parking_ws mux_ws drive_ws bringup_ws; do
  if [[ -f "$_tofu_root/$_ws/install/local_setup.bash" ]]; then
    source "$_tofu_root/$_ws/install/local_setup.bash"
  else
    echo "빌드 안 됨: $_ws (./build_all.sh)" >&2
  fi
done
unset _ws _tofu_root
