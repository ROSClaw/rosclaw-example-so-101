from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path


def normalize_calibration_dir(raw_path: str | Path) -> Path:
    path = Path(raw_path).expanduser()
    if path.suffix == '.json':
        return path.parent
    return path


def coerce_max_relative_target(value):
    if value is None:
        return None
    if isinstance(value, Mapping):
        return {str(key): float(limit) for key, limit in value.items()}
    if isinstance(value, bool):
        raise TypeError('max_relative_target must be numeric or a mapping of joint limits.')
    if isinstance(value, (int, float)):
        numeric_value = float(value)
        return None if abs(numeric_value) <= 1e-9 else numeric_value
    raise TypeError(
        f'max_relative_target must be numeric or a mapping of joint limits. Got {type(value).__name__}.'
    )
