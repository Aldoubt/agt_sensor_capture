#!/usr/bin/env bash
set -euo pipefail
repo="${1:-src/livox_ros_driver2}"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -d "$repo" ]]; then
  echo "livox_ros_driver2 directory not found: $repo" >&2
  exit 2
fi
if [[ ! -f "$repo/package_ROS2.xml" ]]; then
  echo "package_ROS2.xml not found in pinned livox_ros_driver2: $repo" >&2
  exit 2
fi

"$root/scripts/apply_livox_patch.sh" "$repo"
