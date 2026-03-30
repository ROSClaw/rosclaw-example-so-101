"""Local camera wrapper for the SO-101 bringup.

This keeps the USB camera device selection in the example repo instead of
requiring a fork of the upstream `so101_ros2` package.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _usb_cam_remappings(camera_name: str) -> list[tuple[str, str]]:
    return [
        ("image_raw", f"{camera_name}/image_raw"),
        ("image_raw/compressed", f"{camera_name}/image_compressed"),
        ("image_raw/compressedDepth", f"{camera_name}/compressedDepth"),
        ("image_raw/theora", f"{camera_name}/image_raw/theora"),
        ("camera_info", f"{camera_name}/camera_info"),
    ]


def generate_launch_description() -> LaunchDescription:
    so101_bringup_share = FindPackageShare("so101_bringup")

    front_camera_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="cam_front",
        namespace="follower",
        output="screen",
        parameters=[
            PathJoinSubstitution([so101_bringup_share, "config", "so101_usb_cam.yaml"]),
            {
                "camera_name": "cam_front",
                "frame_id": "cam_front",
                "video_device": LaunchConfiguration("front_camera_video_device"),
            },
        ],
        remappings=_usb_cam_remappings("cam_front"),
    )

    side_camera_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="cam_side",
        namespace="static_camera",
        output="screen",
        parameters=[
            PathJoinSubstitution([so101_bringup_share, "config", "so101_realsense2.yaml"]),
            {
                "camera_name": "cam_side",
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "front_camera_video_device",
                default_value="/dev/video2",
                description="Video device path for the SO-101 front USB camera.",
            ),
            front_camera_node,
            side_camera_node,
        ]
    )
