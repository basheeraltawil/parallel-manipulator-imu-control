"""Minimal stand-in for ament_index_python, used ONLY when ROS 2 is not sourced.

It lets `xacro` resolve $(find pm3rrr_description) straight from the source
tree so students can generate/validate the URDF without a ROS installation.
"""
import os
from pathlib import Path

_SRC = Path(__file__).resolve().parents[4] / "ros2_ws" / "src"


class PackageNotFoundError(KeyError):
    pass


def get_package_share_directory(name):
    for prefix in filter(None, os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)):
        p = Path(prefix) / "share" / name
        if p.is_dir():
            return str(p)
    p = _SRC / name
    if (p / "package.xml").exists():
        return str(p)
    raise PackageNotFoundError(name)


def get_package_prefix(name):
    return str(Path(get_package_share_directory(name)).parents[1])
