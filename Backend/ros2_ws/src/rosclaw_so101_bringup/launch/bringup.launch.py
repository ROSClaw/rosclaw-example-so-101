"""Launch ROSClaw together with the SO-101 follower control stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _launch_arguments(*names):
    return {name: LaunchConfiguration(name) for name in names}.items()


def generate_launch_description() -> LaunchDescription:
    follower_calibration_default = PathJoinSubstitution(
        [
            EnvironmentVariable("HOME"),
            ".cache",
            "huggingface",
            "lerobot",
            "calibration",
            "robots",
            "so101_follower",
        ]
    )

    rosclaw_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rosclaw_bringup"), "launch", "rosclaw.launch.py"]
            )
        ),
        launch_arguments=_launch_arguments(
            "use_sim_time",
            "platform",
            "safety_config",
            "discovery_config",
            "perception_config",
            "rosbridge",
            "webrtc",
            "relay",
            "perception",
            "robot_namespace",
            "signaling_url",
            "robot_token",
            "robot_key",
            "robot_id",
        ),
    )

    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rosclaw_so101_bringup"), "launch", "so101_cameras.launch.py"]
            )
        ),
        condition=IfCondition(LaunchConfiguration("cameras")),
        launch_arguments=_launch_arguments("front_camera_video_device"),
    )

    follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("so101_follower_control"),
                    "launch",
                    "follower_control.launch.py",
                ]
            )
        ),
        launch_arguments=_launch_arguments(
            "port",
            "id",
            "calibration_dir",
            "use_degrees",
            "max_relative_target",
            "disable_torque_on_disconnect",
            "publish_rate",
            "model",
            "rviz",
            "rviz_config",
            "cartesian_goal_enabled",
            "cartesian_goal_topic",
            "cartesian_goal_duration_sec",
            "cartesian_base_frame",
            "cartesian_tool_frame",
            "resolved_goal_topic",
            "resolve_cartesian_goal_service",
            "move_to_cartesian_goal_service",
            "convert_cartesian_coordinates_service",
            "agent_bridge_enabled",
            "agent_state_topic",
            "agent_state_publish_rate",
            "gripper_command_topic",
            "open_gripper_service",
            "close_gripper_service",
            "gripper_closed_position",
            "gripper_open_position",
            "gripper_max_effort",
            "pose_registry_enabled",
            "named_pose_catalog_topic",
            "named_pose_file",
            "pose_registry_publish_rate",
            "pose_registry_move_duration_sec",
            "named_pose_match_tolerance_rad",
            "workspace_x_min",
            "workspace_x_max",
            "workspace_y_min",
            "workspace_y_max",
            "workspace_z_min",
            "workspace_z_max",
            "max_cartesian_step_m",
            "max_joint_delta_rad",
            "max_best_effort_position_error_m",
            "safe_orientation_x",
            "safe_orientation_y",
            "safe_orientation_z",
            "safe_orientation_w",
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock for ROSClaw nodes.",
            ),
            DeclareLaunchArgument(
                "platform",
                default_value="generic",
                description="ROSClaw platform preset.",
            ),
            DeclareLaunchArgument(
                "safety_config",
                default_value="",
                description="Optional ROSClaw safety config override.",
            ),
            DeclareLaunchArgument(
                "discovery_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("rosclaw_so101_bringup"), "config", "discovery.yaml"]
                ),
                description="ROSClaw discovery config override.",
            ),
            DeclareLaunchArgument(
                "perception_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("rosclaw_so101_bringup"), "config", "perception.yaml"]
                ),
                description="Optional ROSClaw perception config override.",
            ),
            DeclareLaunchArgument(
                "rosbridge",
                default_value="true",
                description="Launch rosbridge for ROSClaw.",
            ),
            DeclareLaunchArgument(
                "webrtc",
                default_value="false",
                description="Launch the ROSClaw WebRTC bridge.",
            ),
            DeclareLaunchArgument(
                "relay",
                default_value="false",
                description="Launch the ROSClaw relay bridge.",
            ),
            DeclareLaunchArgument(
                "perception",
                default_value="true",
                description="Launch the ROSClaw perception node.",
            ),
            DeclareLaunchArgument(
                "cameras",
                default_value="true",
                description="Launch the SO-101 camera stack.",
            ),
            DeclareLaunchArgument(
                "front_camera_video_device",
                default_value="/dev/video2",
                description="Video device path for the SO-101 front USB camera.",
            ),
            DeclareLaunchArgument(
                "robot_namespace",
                default_value="",
                description="Optional namespace for ROSClaw nodes.",
            ),
            DeclareLaunchArgument(
                "signaling_url",
                default_value="ws://localhost:8443",
                description="ROSClaw signaling server URL.",
            ),
            DeclareLaunchArgument(
                "robot_token",
                default_value="",
                description="ROSClaw signaling token.",
            ),
            DeclareLaunchArgument(
                "robot_key",
                default_value="",
                description="ROSClaw signaling key.",
            ),
            DeclareLaunchArgument(
                "robot_id",
                default_value="robot_1",
                description="ROSClaw robot identifier for signaling.",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="/dev/ttyACM0",
                description="SO-101 follower serial port.",
            ),
            DeclareLaunchArgument(
                "id",
                default_value="atr-follower",
                description="SO-101 follower hardware identifier.",
            ),
            DeclareLaunchArgument(
                "calibration_dir",
                default_value=follower_calibration_default,
                description="SO-101 calibration directory.",
            ),
            DeclareLaunchArgument(
                "use_degrees",
                default_value="true",
                description="Use degree-based commands for the follower bridge.",
            ),
            DeclareLaunchArgument(
                "max_relative_target",
                default_value="10",
                description="Maximum relative target delta for the follower bridge.",
            ),
            DeclareLaunchArgument(
                "disable_torque_on_disconnect",
                default_value="true",
                description="Disable torque if the follower disconnects.",
            ),
            DeclareLaunchArgument(
                "publish_rate",
                default_value="50.0",
                description="Follower state publish rate in Hz.",
            ),
            DeclareLaunchArgument(
                "model",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("so101_description"), "urdf", "so101_new_calib.urdf.xacro"]
                ),
                description="SO-101 follower model xacro path.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Launch RViz for the follower stack.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("so101_follower_control"),
                        "rviz",
                        "follower_control.rviz",
                    ]
                ),
                description="RViz config for the follower stack.",
            ),
            DeclareLaunchArgument(
                "cartesian_goal_enabled",
                default_value="true",
                description="Launch the follower cartesian goal helper node.",
            ),
            DeclareLaunchArgument(
                "cartesian_goal_topic",
                default_value="cartesian_goal",
                description="Follower cartesian goal topic.",
            ),
            DeclareLaunchArgument(
                "cartesian_goal_duration_sec",
                default_value="3.0",
                description="Follower cartesian goal execution duration.",
            ),
            DeclareLaunchArgument(
                "cartesian_base_frame",
                default_value="follower/base_link",
                description="Follower cartesian planning base frame.",
            ),
            DeclareLaunchArgument(
                "cartesian_tool_frame",
                default_value="follower/gripper_frame_link",
                description="Follower cartesian planning tool frame.",
            ),
            DeclareLaunchArgument(
                "resolved_goal_topic",
                default_value="resolved_cartesian_goal",
                description="Follower resolved Cartesian goal metadata topic.",
            ),
            DeclareLaunchArgument(
                "resolve_cartesian_goal_service",
                default_value="resolve_cartesian_goal",
                description="Follower Cartesian resolution service name.",
            ),
            DeclareLaunchArgument(
                "move_to_cartesian_goal_service",
                default_value="move_to_cartesian_goal",
                description="Follower Cartesian move service name.",
            ),
            DeclareLaunchArgument(
                "convert_cartesian_coordinates_service",
                default_value="convert_cartesian_coordinates",
                description="Follower Cartesian unit conversion service name.",
            ),
            DeclareLaunchArgument(
                "agent_bridge_enabled",
                default_value="true",
                description="Launch the SO-101 agent bridge node.",
            ),
            DeclareLaunchArgument(
                "agent_state_topic",
                default_value="agent_state",
                description="Follower agent state topic.",
            ),
            DeclareLaunchArgument(
                "agent_state_publish_rate",
                default_value="2.0",
                description="Follower agent state publish rate in Hz.",
            ),
            DeclareLaunchArgument(
                "gripper_command_topic",
                default_value="gripper_command",
                description="Normalized gripper command topic: 0.0 closed, 1.0 open.",
            ),
            DeclareLaunchArgument(
                "open_gripper_service",
                default_value="open_gripper",
                description="Follower open-gripper Trigger service name.",
            ),
            DeclareLaunchArgument(
                "close_gripper_service",
                default_value="close_gripper",
                description="Follower close-gripper Trigger service name.",
            ),
            DeclareLaunchArgument(
                "gripper_closed_position",
                default_value="0.0",
                description="Raw gripper controller position used for fully closed.",
            ),
            DeclareLaunchArgument(
                "gripper_open_position",
                default_value="1.0",
                description="Raw gripper controller position used for fully open.",
            ),
            DeclareLaunchArgument(
                "gripper_max_effort",
                default_value="10.0",
                description="Gripper max effort used by the agent bridge.",
            ),
            DeclareLaunchArgument(
                "pose_registry_enabled",
                default_value="true",
                description="Launch the SO-101 named pose registry node.",
            ),
            DeclareLaunchArgument(
                "named_pose_catalog_topic",
                default_value="named_pose_catalog",
                description="Named pose catalog topic published by the registry.",
            ),
            DeclareLaunchArgument(
                "named_pose_file",
                default_value=PathJoinSubstitution(
                    [EnvironmentVariable("HOME"), ".ros", "so101_follower", "named_poses.yaml"]
                ),
                description="Host-local YAML file for persisted named poses.",
            ),
            DeclareLaunchArgument(
                "pose_registry_publish_rate",
                default_value="1.0",
                description="Named pose catalog publish rate in Hz.",
            ),
            DeclareLaunchArgument(
                "pose_registry_move_duration_sec",
                default_value="3.0",
                description="Default joint trajectory duration when moving to a named pose.",
            ),
            DeclareLaunchArgument(
                "named_pose_match_tolerance_rad",
                default_value="0.08",
                description="Joint-space tolerance used to label the current named pose.",
            ),
            DeclareLaunchArgument(
                "workspace_x_min",
                default_value="-0.3",
                description="Cartesian workspace minimum X in meters.",
            ),
            DeclareLaunchArgument(
                "workspace_x_max",
                default_value="0.5",
                description="Cartesian workspace maximum X in meters.",
            ),
            DeclareLaunchArgument(
                "workspace_y_min",
                default_value="-0.4",
                description="Cartesian workspace minimum Y in meters.",
            ),
            DeclareLaunchArgument(
                "workspace_y_max",
                default_value="0.4",
                description="Cartesian workspace maximum Y in meters.",
            ),
            DeclareLaunchArgument(
                "workspace_z_min",
                default_value="0.0",
                description="Cartesian workspace minimum Z in meters.",
            ),
            DeclareLaunchArgument(
                "workspace_z_max",
                default_value="0.5",
                description="Cartesian workspace maximum Z in meters.",
            ),
            DeclareLaunchArgument(
                "max_cartesian_step_m",
                default_value="0.25",
                description="Maximum safe Cartesian move distance from the current tool pose.",
            ),
            DeclareLaunchArgument(
                "max_joint_delta_rad",
                default_value="1.1",
                description="Maximum allowed per-joint delta for a resolved Cartesian goal.",
            ),
            DeclareLaunchArgument(
                "max_best_effort_position_error_m",
                default_value="0.08",
                description="Maximum allowed distance between the requested and resolved Cartesian goal.",
            ),
            DeclareLaunchArgument(
                "safe_orientation_x",
                default_value="0.0",
                description="Fixed safe tool orientation quaternion X component.",
            ),
            DeclareLaunchArgument(
                "safe_orientation_y",
                default_value="0.707107",
                description="Fixed safe tool orientation quaternion Y component.",
            ),
            DeclareLaunchArgument(
                "safe_orientation_z",
                default_value="0.0",
                description="Fixed safe tool orientation quaternion Z component.",
            ),
            DeclareLaunchArgument(
                "safe_orientation_w",
                default_value="0.707107",
                description="Fixed safe tool orientation quaternion W component.",
            ),
            cameras_launch,
            rosclaw_launch,
            follower_launch,
        ]
    )
