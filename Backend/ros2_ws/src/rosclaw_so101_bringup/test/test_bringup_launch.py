from __future__ import annotations

import importlib.util
import os
from pathlib import Path


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
    assert 'FindPackageShare("so101_follower_control")' in contents
