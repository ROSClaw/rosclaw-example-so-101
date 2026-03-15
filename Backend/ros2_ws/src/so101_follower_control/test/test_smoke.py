from pathlib import Path


def test_launch_file_exists():
    root = Path(__file__).resolve().parents[1]
    assert (root / 'launch' / 'follower_control.launch.py').is_file()


def test_readme_exists():
    root = Path(__file__).resolve().parents[1]
    assert (root / 'README.md').is_file()


def test_rviz_config_exists():
    root = Path(__file__).resolve().parents[1]
    assert (root / 'rviz' / 'follower_control.rviz').is_file()
