#!/usr/bin/env bash
set -eo pipefail

# Keep the build on the system Python/ROS environment without depending on a
# Conda-initialized shell or mutating the parent shell session.
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_PYTHON_EXE _CE_CONDA _CE_M
export CONDA_SHLVL=0
export PATH="/usr/bin:/bin:${PATH:-}"

source /opt/ros/humble/setup.bash
colcon build --cmake-clean-cache
