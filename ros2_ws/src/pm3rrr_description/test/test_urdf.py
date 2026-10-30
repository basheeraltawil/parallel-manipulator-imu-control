"""colcon test: the xacro expands, is a valid tree, and every mesh exists.

The heavier geometric checks (loop closure vs. kinematics, inertia) live in
tools/cad_to_urdf/step4_validate_urdf.py and run in the Python CI job.
"""
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

PKG = Path(os.environ.get("PM3RRR_DESCRIPTION_SHARE", Path(__file__).resolve().parents[1]))
XACRO = PKG / "urdf" / "pm3rrr.urdf.xacro"


def expand(*args):
    out = subprocess.run(["xacro", str(XACRO), *args], capture_output=True, text=True, check=True)
    return ET.fromstring(out.stdout)


@pytest.mark.parametrize("args", [(), ("use_gazebo:=true", "use_ros2_control:=true")])
def test_xacro_expands_to_a_tree(args):
    root = expand(*args)
    links = {l.get("name") for l in root.findall("link")}
    joints = root.findall("joint")
    children = [j.find("child").get("link") for j in joints]
    assert len(children) == len(set(children)), "a link has two parents -> not a tree"
    assert set(children) | {"world"} == links
    movable = {j.get("name") for j in joints if j.get("type") != "fixed"}
    assert movable == {f"leg{i}_{k}_joint" for i in (1, 2, 3) for k in ("motor", "elbow")} | {"leg1_platform_joint"}


def test_meshes_exist():
    root = expand()
    for m in root.iter("mesh"):
        rel = m.get("filename").replace("package://pm3rrr_description/", "")
        assert (PKG / rel).exists(), rel


def test_gazebo_variant_has_loop_closures_and_ros2_control():
    root = expand("use_gazebo:=true", "use_ros2_control:=true")
    sdf_joints = [j.get("name") for g in root.findall("gazebo") for j in g.findall("joint")]
    assert sorted(sdf_joints) == ["leg2_platform_joint", "leg3_platform_joint"]
    rc = root.find("ros2_control")
    assert rc is not None
    cmd = [j.get("name") for j in rc.findall("joint") if j.find("command_interface") is not None]
    assert cmd == ["leg1_motor_joint", "leg2_motor_joint", "leg3_motor_joint"]


@pytest.mark.skipif(shutil.which("check_urdf") is None, reason="liburdfdom-tools not installed")
def test_check_urdf(tmp_path):
    out = subprocess.run(["xacro", str(XACRO)], capture_output=True, text=True, check=True).stdout
    f = tmp_path / "pm3rrr.urdf"
    f.write_text(out)
    res = subprocess.run(["check_urdf", str(f)], capture_output=True, text=True)
    assert res.returncode == 0, res.stdout + res.stderr
