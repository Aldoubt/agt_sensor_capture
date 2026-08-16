#!/usr/bin/env bash
set -euo pipefail
repo="${1:?usage: apply_livox_patch.sh PATH_TO_LIVOX_ROS_DRIVER2}"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python3 "$root/scripts/patch_livox_ros2.py" "$repo"
