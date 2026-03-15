  conda deactivate
  export PATH=/usr/bin:/bin:$PATH
  source /opt/ros/humble/setup.bash
  colcon build --cmake-clean-cache
