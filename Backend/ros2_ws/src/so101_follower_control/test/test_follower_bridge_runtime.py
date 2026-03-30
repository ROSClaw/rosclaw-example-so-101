from __future__ import annotations

from pathlib import Path

import pytest

from so101_follower_control.follower_bridge_runtime import (
    coerce_max_relative_target,
    normalize_calibration_dir,
)


def test_coerce_max_relative_target_accepts_integer_and_returns_float():
    assert coerce_max_relative_target(10) == 10.0


def test_coerce_max_relative_target_treats_zero_as_disabled():
    assert coerce_max_relative_target(0) is None
    assert coerce_max_relative_target(0.0) is None


def test_coerce_max_relative_target_preserves_mapping_values():
    assert coerce_max_relative_target({'gripper': 5, 'wrist_roll': 7.5}) == {
        'gripper': 5.0,
        'wrist_roll': 7.5,
    }


def test_coerce_max_relative_target_rejects_boolean():
    with pytest.raises(TypeError):
        coerce_max_relative_target(True)


def test_normalize_calibration_dir_passes_directory_through():
    path = Path('/tmp/calibration')
    assert normalize_calibration_dir(path) == path


def test_normalize_calibration_dir_reduces_json_file_to_parent():
    assert normalize_calibration_dir('/tmp/calibration/atr-follower.json') == Path('/tmp/calibration')
