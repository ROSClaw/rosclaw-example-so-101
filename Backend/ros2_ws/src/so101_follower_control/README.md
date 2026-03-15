# so101_follower_control

Thin ROS2 bringup package for the physical LeRobot SO-101 follower arm.

This package reuses the existing `so101_ros2` backend:

- `so101_ros2_bridge` for LeRobot hardware access
- `so101_hardware_interface` for the `ros2_control` bridge
- `so101_controller` for standard ROS controllers

## Public control interfaces

- `/follower/arm_controller/follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`)
- `/follower/gripper_controller/gripper_cmd` (`control_msgs/action/GripperCommand`)
- `/follower/joint_states` (`sensor_msgs/msg/JointState`)
- `/follower/cartesian_goal` (`geometry_msgs/msg/PoseStamped`)
- `/follower/agent_state` (`std_msgs/msg/String`) JSON summary of current joints, gripper, and tool pose
- `/follower/gripper_command` (`std_msgs/msg/Float64`) normalized gripper open fraction, where `0.0 = closed` and `1.0 = open`
- `/follower/open_gripper` (`std_srvs/srv/Trigger`) explicit one-shot open command for agents
- `/follower/close_gripper` (`std_srvs/srv/Trigger`) explicit one-shot close command for agents

Joint order for arm trajectories:

1. `shoulder_pan`
2. `shoulder_lift`
3. `elbow_flex`
4. `wrist_flex`
5. `wrist_roll`

The gripper is controlled through the gripper action controller, not as part of the arm trajectory.
For agent use, prefer `/follower/open_gripper` and `/follower/close_gripper` over the raw action.
Cartesian goals command the pose of `follower/gripper_frame_link` and are converted into single-point joint trajectories for the arm controller.

## Bringup

```bash
ros2 launch so101_follower_control follower_control.launch.py port:=/dev/ttyACM0
```

Launch with the package RViz layout:

```bash
ros2 launch so101_follower_control follower_control.launch.py port:=/dev/ttyACM0 rviz:=true
```

Optional launch arguments:

- `id` default: `Tzili`
- `calibration_dir` default: upstream `so101_ros2_bridge/config/calibration`
- `use_degrees` default: `true`
- `max_relative_target` default: `10`
- `disable_torque_on_disconnect` default: `true`
- `publish_rate` default: `50.0`
- `rviz` default: `false`
- `rviz_config` default: package RViz config at `rviz/follower_control.rviz`
- `cartesian_goal_enabled` default: `true`
- `cartesian_goal_topic` default: `cartesian_goal` (resolves to `/follower/cartesian_goal`)
- `cartesian_goal_duration_sec` default: `3.0`
- `cartesian_base_frame` default: `follower/base_link`
- `cartesian_tool_frame` default: `follower/gripper_frame_link`
- `agent_bridge_enabled` default: `true`
- `agent_state_topic` default: `agent_state` (resolves to `/follower/agent_state`)
- `agent_state_publish_rate` default: `2.0`
- `gripper_command_topic` default: `gripper_command` (resolves to `/follower/gripper_command`)
- `open_gripper_service` default: `open_gripper`
- `close_gripper_service` default: `close_gripper`
- `gripper_closed_position` default: `0.0`
- `gripper_open_position` default: `1.0`
- `gripper_max_effort` default: `10.0`

## Example clients

Send a single-point arm trajectory:

```bash
ros2 run so101_follower_control joint_trajectory_example --ros-args -p positions:="[0.0,0.0,0.0,0.0,0.0]"
```

Open the gripper:

```bash
ros2 run so101_follower_control gripper_command_example --ros-args -p position:=1.0
```

Open the gripper through the agent-facing service:

```bash
ros2 service call /follower/open_gripper std_srvs/srv/Trigger
```

Close the gripper through the normalized command topic:

```bash
ros2 topic pub --once /follower/gripper_command std_msgs/msg/Float64 "{data: 0.0}"
```

Watch joint states:

```bash
ros2 run so101_follower_control joint_state_echo
```

Send a Cartesian goal to the gripper frame:

```bash
ros2 topic pub --once /follower/cartesian_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: follower/base_link}, pose: {position: {x: 0.391361, y: 0.0, z: 0.226470}, orientation: {x: 0.017206, y: 0.706894, z: 0.017214, w: 0.706900}}}"
```

The Cartesian goal node is a lightweight IK bridge. It does not do collision checking or path planning, and it controls the five arm joints only.
