from __future__ import annotations

import importlib.util
from pathlib import Path

from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def _load_launch_module():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / 'launch'
        / 'follower_control.launch.py'
    )
    spec = importlib.util.spec_from_file_location('so101_follower_control_launch', launch_path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_follower_bridge_parameter_overrides_use_typed_values():
    module = _load_launch_module()
    overrides = module._follower_bridge_parameter_overrides(
        LaunchConfiguration('port'),
        LaunchConfiguration('id'),
        LaunchConfiguration('calibration_dir'),
        LaunchConfiguration('use_degrees'),
        LaunchConfiguration('max_relative_target'),
        LaunchConfiguration('disable_torque_on_disconnect'),
        LaunchConfiguration('publish_rate'),
    )

    assert isinstance(overrides['port'], ParameterValue)
    assert overrides['port'].value_type is str
    assert overrides['calibration_dir'].value_type is str
    assert overrides['use_degrees'].value_type is bool
    assert overrides['max_relative_target'].value_type is int
    assert overrides['disable_torque_on_disconnect'].value_type is bool
    assert overrides['publish_rate'].value_type is float
    assert overrides['type'] == 'follower'
