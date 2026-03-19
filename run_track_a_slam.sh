#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PIDS=()

count_node_instances() {
  local node_name="$1"
  ros2 node list --no-daemon 2>/dev/null | awk -v node="$node_name" '$0 == node {count++} END {print count + 0}'
}

ensure_not_running() {
  local node_name="$1"
  local count
  count="$(count_node_instances "$node_name")"
  if [ "$count" -gt 0 ]; then
    echo "Node already running: $node_name ($count instance(s)). Stop existing Track A nodes first." >&2
    exit 1
  fi
}

cleanup() {
  trap - EXIT INT TERM

  local sig
  local pid
  for sig in INT TERM KILL; do
    for pid in "${PIDS[@]:-}"; do
      if kill -0 "$pid" 2>/dev/null; then
        kill "-$sig" -- "-$pid" 2>/dev/null || true
      fi
    done
    for pid in "${PIDS[@]:-}"; do
      wait "$pid" 2>/dev/null || true
    done
    sleep 0.2
  done
}

on_signal() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM

cd "$SCRIPT_DIR"

ensure_not_running /time_sync_bridge
ensure_not_running /lidar_scan_bridge
ensure_not_running /slam_toolbox

setsid python3 time_sync_bridge.py &
PIDS+=("$!")

setsid python3 lidar_scan_bridge.py &
PIDS+=("$!")

sleep 2

setsid ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  --params-file "$SCRIPT_DIR/slam_toolbox_go2_async.yaml" &
PIDS+=("$!")

wait
