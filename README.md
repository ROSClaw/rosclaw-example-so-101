# ROSClaw Example SO-101

This repository packages the ROSClaw and OpenClaw bringup for a physical SO-101 follower arm.

## Layout

```text
Backend/
  ros2_ws/
    src/
      rosclaw_so101_bringup/     # ROSClaw + SO-101 combined bringup package
      so101_follower_control/    # SO-101 follower control helpers and agent bridge
      rosclaw-ros2/              # git submodule
      so101_ros2/                # git submodule
```

## What Is Included

- `so101_follower_control` contains the physical SO-101 follower bringup, Cartesian goal helper, and agent bridge.
- `rosclaw_so101_bringup` launches ROSClaw together with the SO-101 follower stack.
- The agent bridge exposes explicit gripper services at `/follower/open_gripper` and `/follower/close_gripper`.
- The ROSClaw bringup package ships the SO-101-specific discovery config.

## Clone With Submodules

```bash
git clone https://github.com/ROSClaw/rosclaw-example-so-101.git
cd rosclaw-example-so-101
git submodule update --init --recursive
```

## Build

Use the system Python 3 package:

```bash
source /opt/ros/humble/setup.bash
pip install 'google-genai>=1.0.0'
cd Backend/ros2_ws
export PATH=/usr/bin:/bin:/usr/sbin:/sbin:$PATH
colcon build --packages-up-to rosclaw_so101_bringup \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  --allow-overriding so101_controller so101_description so101_ros2_bridge
```

## Launch

```bash
source /opt/ros/humble/setup.bash
source Backend/ros2_ws/install/setup.bash
export GEMINI_API_KEY=your-api-key
ros2 launch rosclaw_so101_bringup bringup.launch.py
```

Default SO-101 follower bringup values:

- `port:=/dev/ttyACM0`
- `id:=atr-follower`
- `calibration_dir:=$HOME/.cache/huggingface/lerobot/calibration/robots/so101_follower/atr-follower.json`
- `cameras:=true`
- `perception:=true`

The combined bringup now launches the SO-101 camera stack and the ROSClaw
perception node by default. It expects the USB wrist camera to publish on
`/follower/cam_front/image_compressed` and fails fast if the Gemini dependency,
`GEMINI_API_KEY`, or startup camera frames are missing.

## Agent-Facing Control Surfaces

- `/follower/agent_state` publishes a JSON summary of joint positions, tool pose, and `gripper_open_fraction`
- `/follower/open_gripper` and `/follower/close_gripper` are explicit one-shot Trigger services for agent use
- `/follower/gripper_command` is a normalized open fraction topic where `0.0 = closed` and `1.0 = open`
- `/follower/cartesian_goal` accepts `geometry_msgs/msg/PoseStamped`
- `/follower/arm_controller/follow_joint_trajectory` remains available for low-level arm motion goals

## Manual Smoke Tests

```bash
ros2 service call /follower/open_gripper std_srvs/srv/Trigger
ros2 service call /follower/close_gripper std_srvs/srv/Trigger
```
