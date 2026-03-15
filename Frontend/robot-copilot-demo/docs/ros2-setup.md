# Remote ROSClaw Setup

This app no longer connects to ROS directly over a user-configured rosbridge
socket. The runtime flow is:

1. the visionOS app starts a local OpenClawKit runtime
2. the runtime loads `rosclaw-swift-plugin`
3. the plugin reads the remote OpenClaw agent endpoint from environment
4. the plugin asks the remote agent for robot snapshots and operation execution
5. the remote OpenClaw agent uses its installed TypeScript `rosclaw-plugin` to
   talk to the SO-101 ROS2 stack on the robot network

## Local App Environment

Launch the app with:

| Variable | Required | Default | Description |
|---|---|---|---|
| `ROSCLAW_AGENT_IP` | yes | none | IP or hostname of the remote OpenClaw agent |
| `ROSCLAW_AGENT_PORT` | no | `18789` | Remote OpenClaw gateway port |
| `ROSCLAW_AGENT_AUTH_TOKEN` | no | none | Token used if the remote agent requires auth |
| `ROSCLAW_AGENT_TLS_FINGERPRINT` | no | none | Enables TLS pinning when set |
| `OPENAI_API_KEY` | no | none | Enables the OpenAI provider for the local embedded runtime |

There is intentionally no in-app UI for editing the remote endpoint.

## Robot Host Requirements

The robot-side network host still needs:

- the remote OpenClaw agent running and reachable from the Vision Pro network
- the existing TypeScript `rosclaw-plugin` installed in that remote agent
- the SO-101 follower stack running
- the `so101_rosclaw_adapter` node exposing the ROSClaw manipulator contract

## ROS2 Bringup

Build the hardware workspace:

```bash
source /opt/ros/humble/setup.bash
cd hardware/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

Launch the follower stack with the ROSClaw adapter:

```bash
ros2 launch so101_rosclaw_adapter so101_rosclaw.launch.py port:=/dev/ttyACM0
```

The adapter provides:

- `/rosclaw/get_manifest`
- `/rosclaw/manifest`
- `/rosclaw/safety_status`
- `/rosclaw/estop`

The remote ROSClaw plugin can still use the raw follower interfaces surfaced by
the same graph:

- `/follower/arm_controller/follow_joint_trajectory`
- `/follower/gripper_controller/gripper_cmd`
- `/follower/cartesian_goal`
- `/follower/joint_states`

## E-stop Semantics

The adapter treats `/rosclaw/estop = true` as a hard stop request:

- cancel active arm and gripper goals
- publish an empty arm trajectory to stop the controller
- optionally call a configured disconnect service for torque-off or bridge
  disconnect behavior
- publish a latched unsafe state on `/rosclaw/safety_status`

`/rosclaw/estop = false` clears the latched state, but this is still software
control only. Hardware/manual estop remains the primary safety mechanism.
