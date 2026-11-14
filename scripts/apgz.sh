#!/usr/bin/env bash
# QGC: default MAVProxy output (udp 14550). MAVROS: udp://:14555@
set -euo pipefail

WORLD="${1:-iris_runway.sdf}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
ROS2_DIR="$(dirname "$WS_DIR")"
ARDUPILOT_DIR="$ROS2_DIR/ardupilot"
GZ_PLUGIN_DIR="$ROS2_DIR/ardupilot_gazebo"

command -v gz >/dev/null || {
  echo "gz not found: run inside ros2-jazzy"
  exit 1
}
[[ -f "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" ]] || {
  echo "ArduPilot not found at $ARDUPILOT_DIR"
  exit 1
}
[[ -d "$GZ_PLUGIN_DIR/build" ]] || {
  echo "ardupilot_gazebo not built at $GZ_PLUGIN_DIR/build"
  exit 1
}

# Gazebo env (set here, non-interactive shells do not read .bashrc)
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH="$GZ_PLUGIN_DIR/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$GZ_PLUGIN_DIR/models:$GZ_PLUGIN_DIR/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

# stop Gazebo when SITL exits or on Ctrl+C
GZ_PID=""
cleanup() {
  [[ -n "$GZ_PID" ]] && kill "$GZ_PID" 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Starting Gazebo: $WORLD"
gz sim -v4 -r "$WORLD" &
GZ_PID=$!
sleep 5

echo "Starting SITL"
cd "$ARDUPILOT_DIR"
python3 Tools/autotest/sim_vehicle.py \
  -v ArduCopter \
  -f gazebo-iris \
  --model JSON \
  --console --map \
  --out udp:127.0.0.1:14555
