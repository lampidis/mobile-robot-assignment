"""Small helpers shared by the navigation integration scenarios."""

from pathlib import Path

from ament_index_python.packages import get_package_prefix


def installed_executable(package: str, executable: str) -> str:
    """Return the installed ROS executable without launching a `ros2 run` wrapper."""
    path = Path(get_package_prefix(package)) / "lib" / package / executable
    if not path.is_file():
        raise FileNotFoundError(f"Build the {package} package first: {path}")
    return str(path)
