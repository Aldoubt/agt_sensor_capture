#!/usr/bin/env bash
set -euo pipefail
repo="${1:?usage: apply_livox_patch.sh PATH_TO_LIVOX_ROS_DRIVER2}"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
patch_file="$root/patches/livox_ros_driver2/0001-agt-shared-timebase.patch"

if git -C "$repo" apply --check "$patch_file"; then
  git -C "$repo" apply "$patch_file"
elif git -C "$repo" apply --reverse --check "$patch_file"; then
  echo "AGT Livox timebase patch already applied"
else
  echo "AGT Livox timebase patch does not apply cleanly" >&2
  exit 2
fi
