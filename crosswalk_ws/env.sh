#!/usr/bin/env bash
# 각 터미널에서: source env.sh
_ws_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
if [[ -f "$_ws_root/install/local_setup.bash" ]]; then
  source "$_ws_root/install/local_setup.bash"
fi
unset _ws_root
