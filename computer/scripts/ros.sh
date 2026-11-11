#!/bin/bash
# ROS 2 bridge natively on Linux (ROS 2 Jazzy/Humble installed). Same actions as ros.ps1.
#
#   ./computer/scripts/ros.sh              build + start the node (Ctrl+C stops)
#   ./computer/scripts/ros.sh move 0.2     move 0.2 m          (from a 2nd terminal)
#   ./computer/scripts/ros.sh rotate 90    rotate o 90°
#   ./computer/scripts/ros.sh status | result | odom | scan | clear
#
# Workspace: $ROBOT_BRIDGE_WS (default ~/robot_bridge_ws), the package is symlinked into its src/.

set -e
computer="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ws="${ROBOT_BRIDGE_WS:-$HOME/robot_bridge_ws}"
distro="${ROS_DISTRO:-$(ls /opt/ros 2> /dev/null | head -1)}"
[ -f "/opt/ros/$distro/setup.bash" ] || { echo "FAIL  ROS 2 not found in /opt/ros"; exit 1; }
set +u; source "/opt/ros/$distro/setup.bash"

ros() {
    [ -f "$ws/install/setup.bash" ] || { echo "FAIL  not built yet, run: $0"; exit 1; }
    source "$ws/install/setup.bash"
    "$@"
}
number() { [[ "$1" =~ ^-?[0-9]+([.,][0-9]+)?$ ]] || { echo "FAIL  needs a number, e.g. 0.5"; exit 1; }; echo "${1/,/.}"; }

case "${1:-start}" in
    move)   v=$(number "$2"); ros ros2 topic pub --once /robot_bridge/move std_msgs/msg/Float32 "{data: $v}" > /dev/null; echo "sent move $v m"; exit ;;
    rotate) v=$(number "$2"); ros ros2 topic pub --once /robot_bridge/rotate std_msgs/msg/Float32 "{data: $v}" > /dev/null; echo "sent rotate $v deg"; exit ;;
    status) ros ros2 topic echo --once /robot_bridge/status; exit ;;
    result) ros ros2 topic echo /robot_bridge/result; exit ;;
    odom)   ros ros2 topic echo --once /odom; exit ;;
    scan)   ros ros2 topic echo --once /scan; exit ;;
    clear)  ros ros2 service call /robot_bridge/clear_queue std_srvs/srv/Trigger; exit ;;
    start)  ;;
    *)      echo "FAIL  unknown action '$1' (see the top of $0)"; exit 1 ;;
esac

mkdir -p "$ws/src"
[ -e "$ws/src/robot_bridge" ] || ln -s "$computer" "$ws/src/robot_bridge"
cd "$ws"
args=()
command -v ninja > /dev/null && args=(--cmake-args -G Ninja)
echo "== colcon build ${args[*]}"
colcon build --packages-select robot_bridge "${args[@]}"
source install/setup.bash

# server address the firmware connects to (robot/main/main.cpp)
main="$computer/../robot/main/main.cpp"
ip=$(grep -oP 'DataExchanger\s+\w+\(\s*"\K[\d.]+' "$main" 2> /dev/null || true)
grep -q 'wifiEnabled\s*=\s*true' "$main" 2> /dev/null || echo "WARN  firmware has wifiEnabled = false: the robot will not connect"
if [ -n "$ip" ] && ! hostname -I | tr ' ' '\n' | grep -qx "$ip"; then
    echo "WARN  robot connects to $ip, this PC has: $(hostname -I)"
fi

echo "== node waiting for the robot. Ctrl+C stops it."
exec ros2 launch robot_bridge robot_bridge.launch.xml
