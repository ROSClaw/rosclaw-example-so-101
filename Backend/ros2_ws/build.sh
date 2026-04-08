#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$SCRIPT_DIR"
COMPAT_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/rosclaw-so101-src.XXXXXX")"
trap 'rm -rf "$COMPAT_ROOT"' EXIT
COMPAT_BASE_PATHS="$(bash "$WORKSPACE_ROOT/scripts/prepare_compat_sources.sh" "$COMPAT_ROOT")"

# Keep the build on the system Python/ROS environment without depending on a
# Conda-initialized shell or mutating the parent shell session.
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_PYTHON_EXE _CE_CONDA _CE_M
export CONDA_SHLVL=0
export PATH="/usr/bin:/bin:${PATH:-}"

ROS_DISTRO_NAME=${ROS_DISTRO:-jazzy}
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
ROSDEP_SKIP_KEYS_VALUE=${ROSDEP_SKIP_KEYS:-topic_based_ros2_control_msgs}

if ! command -v rosdep >/dev/null 2>&1; then
  sudo apt update
  sudo apt install -y python3-rosdep
fi

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init || true
fi

rosdep update
if [ -t 0 ] && [ -t 1 ]; then
  rosdep install --from-paths "$COMPAT_BASE_PATHS" --ignore-src -r -y \
    --skip-keys "$ROSDEP_SKIP_KEYS_VALUE"
else
  echo "Skipping rosdep install in a non-interactive shell." >&2
  echo "Run this manually if dependencies are missing:" >&2
  echo "  rosdep install --from-paths \"$COMPAT_BASE_PATHS\" --ignore-src -r -y --skip-keys \"$ROSDEP_SKIP_KEYS_VALUE\"" >&2
fi

colcon --log-base "$WORKSPACE_ROOT/log" build \
  --base-paths "$COMPAT_BASE_PATHS" \
  --build-base "$WORKSPACE_ROOT/build" \
  --install-base "$WORKSPACE_ROOT/install" \
  --cmake-clean-cache \
  --event-handlers console_direct+
