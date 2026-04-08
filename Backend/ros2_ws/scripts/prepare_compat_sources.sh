#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
UPSTREAM_SRC_ROOT="$WORKSPACE_ROOT/src"
COMPAT_RECORDER_SRC="$WORKSPACE_ROOT/compat/system_data_recorder"
COMPAT_TELEOP_SRC="$WORKSPACE_ROOT/compat/so101_teleop"
OUTPUT_ROOT="${1:-}"

if [ -z "$OUTPUT_ROOT" ]; then
  echo "Usage: $0 <output-root>" >&2
  exit 1
fi

if [ ! -d "$UPSTREAM_SRC_ROOT" ]; then
  echo "Missing workspace source tree: $UPSTREAM_SRC_ROOT" >&2
  exit 1
fi

if [ ! -d "$COMPAT_RECORDER_SRC" ]; then
  echo "Missing compatibility package: $COMPAT_RECORDER_SRC" >&2
  exit 1
fi

if [ ! -d "$COMPAT_TELEOP_SRC" ]; then
  echo "Missing compatibility package: $COMPAT_TELEOP_SRC" >&2
  exit 1
fi

rm -rf "$OUTPUT_ROOT"
mkdir -p "$OUTPUT_ROOT"

cp -a "$UPSTREAM_SRC_ROOT" "$OUTPUT_ROOT/src"
rm -rf "$OUTPUT_ROOT/src/so101_ros2/ros2_externals/system_data_recorder"
mkdir -p "$OUTPUT_ROOT/src/so101_ros2/ros2_externals"
cp -a "$COMPAT_RECORDER_SRC" "$OUTPUT_ROOT/src/so101_ros2/ros2_externals/system_data_recorder"
rm -rf "$OUTPUT_ROOT/src/so101_ros2/so101_teleop"
cp -a "$COMPAT_TELEOP_SRC" "$OUTPUT_ROOT/src/so101_ros2/so101_teleop"

printf '%s\n' "$OUTPUT_ROOT/src"
