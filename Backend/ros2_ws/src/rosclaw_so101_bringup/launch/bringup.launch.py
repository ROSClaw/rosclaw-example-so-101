"""Launch ROSClaw together with the SO-101 follower control stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
            "atr-follower.json",
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
            "agent_bridge_enabled",
            "agent_state_topic",
            "agent_state_publish_rate",
            "gripper_command_topic",
            "open_gripper_service",
            "close_gripper_service",
            "gripper_closed_position",
            "gripper_open_position",
            "gripper_max_effort",
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
                default_value="",
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
                default_value="false",
                description="Launch the ROSClaw perception node.",
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
                description="SO-101 calibration path.",
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
            rosclaw_launch,
            follower_launch,
        ]
    )
