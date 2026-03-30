from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import re


def _load_launch_module():
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "bringup.launch.py"
    spec = importlib.util.spec_from_file_location("rosclaw_so101_bringup_launch", launch_path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_generate_launch_description_imports(monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", os.environ.get("ROS_LOG_DIR", "/tmp/ros_logs"))
    module = _load_launch_module()
    description = module.generate_launch_description()
    assert description.entities


def test_launch_file_includes_both_stacks():
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "bringup.launch.py"
    contents = launch_path.read_text()
    assert 'FindPackageShare("rosclaw_bringup")' in contents
    assert 'FindPackageShare("rosclaw_so101_bringup"), "launch", "so101_cameras.launch.py"' in contents
    assert 'FindPackageShare("so101_follower_control")' in contents


def test_launch_uses_calibration_directory_default():
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "bringup.launch.py"
    contents = launch_path.read_text()
    assert "so101_follower" in contents
    assert "atr-follower.json" not in contents


def test_launch_forwards_cartesian_services():
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "bringup.launch.py"
    contents = launch_path.read_text()
    assert '"cartesian_goal_enabled"' in contents
    assert '"resolve_cartesian_goal_service"' in contents
    assert '"move_to_cartesian_goal_service"' in contents
    assert '"convert_cartesian_coordinates_service"' in contents


def test_launch_defaults_enable_cameras_and_perception():
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "bringup.launch.py"
    contents = launch_path.read_text()
    assert re.search(r'DeclareLaunchArgument\(\s*"cameras",\s*default_value="true"', contents)
    assert re.search(r'DeclareLaunchArgument\(\s*"perception",\s*default_value="true"', contents)
    assert re.search(
        r'DeclareLaunchArgument\(\s*"front_camera_video_device",\s*default_value="/dev/video2"',
        contents,
    )
    assert 'FindPackageShare("rosclaw_so101_bringup"), "config", "perception.yaml"' in contents
