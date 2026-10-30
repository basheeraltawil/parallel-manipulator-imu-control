#!/usr/bin/env python3
"""STEP 4 of the CAD -> URDF pipeline: prove the model is right.

Checks (all must pass before the model is used in Gazebo/RViz/industry):
  1. xacro expands for every argument combination
  2. the URDF parses and is a tree with the expected joints
  3. every mesh file exists and every link has positive mass / valid inertia
  4. *loop closure*: for random reachable poses, put the joint values from
     the inverse kinematics into the URDF, do forward kinematics through the
     TREE and check that legs 2 and 3 really end on the platform
     (error must be < 1 micrometre) and that platform_center == pose.

Usage:  python3 tools/cad_to_urdf/step4_validate_urdf.py [--write-urdf out.urdf]
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "ros2_ws" / "src"
XACRO = SRC / "pm3rrr_description" / "urdf" / "pm3rrr.urdf.xacro"
sys.path.insert(0, str(SRC / "pm3rrr_core"))

from pm3rrr_core import Pm3rrrKinematics  # noqa: E402


def expand_xacro(args=()) -> str:
    env = dict(os.environ)
    try:
        import ament_index_python  # noqa: F401  (real ROS 2 sourced)
    except ImportError:
        env["PYTHONPATH"] = os.pathsep.join(
            [str(ROOT / "tools" / "cad_to_urdf" / "_ros_shim"), env.get("PYTHONPATH", "")])
    out = subprocess.run(["xacro", str(XACRO), *args], capture_output=True, text=True, env=env)
    if out.returncode != 0:
        raise RuntimeError(out.stderr)
    return out.stdout


def resolve_package_uris(urdf: str) -> str:
    return urdf.replace("package://pm3rrr_description", str(SRC / "pm3rrr_description"))


def check(cond, msg):
    print(("  [PASS] " if cond else "  [FAIL] ") + msg)
    if not cond:
        check.failed = True


check.failed = False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--write-urdf", help="also save the expanded URDF here")
    ap.add_argument("--samples", type=int, default=200)
    a = ap.parse_args()

    import yourdfpy

    print("1) xacro expansion")
    variants = {"plain": (), "gazebo": ("use_gazebo:=true", "use_ros2_control:=true")}
    urdfs = {}
    for k, args in variants.items():
        try:
            urdfs[k] = expand_xacro(args)
            check(True, f"xacro {k} ({len(urdfs[k].splitlines())} lines)")
        except RuntimeError as e:
            check(False, f"xacro {k}: {e}")
            sys.exit(1)
    if a.write_urdf:
        Path(a.write_urdf).write_text(urdfs["plain"])

    print("2) structure")
    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as f:
        f.write(resolve_package_uris(urdfs["plain"]))
    robot = yourdfpy.URDF.load(f.name, load_meshes=False, build_scene_graph=True)
    expected = {f"leg{i}_{j}_joint" for i in (1, 2, 3) for j in ("motor", "elbow")} | {"leg1_platform_joint"}
    check(set(robot.actuated_joint_names) == expected, f"movable joints = {sorted(expected)}")
    check(robot.base_link == "world", "root link is 'world'")
    gz = urdfs["gazebo"]
    check(gz.count("<parent>leg2_distal_link</parent>") == 1 and gz.count("<parent>leg3_distal_link</parent>") == 1,
          "gazebo variant contains the 2 loop-closure joints")
    check("gazebo_ros2_control/GazeboSystem" in gz, "gazebo variant contains ros2_control hardware")

    print("3) meshes and inertia")
    for link in robot.robot.links:
        for g in [v.geometry for v in link.visuals] + [c.geometry for c in link.collisions]:
            if g.mesh is not None:
                check(Path(g.mesh.filename).exists(), f"{link.name}: mesh {Path(g.mesh.filename).name}")
        if link.inertial is not None:
            I = link.inertial.inertia
            ev = np.linalg.eigvalsh(I)
            tri = ev[0] + ev[1] >= ev[2] * (1 - 1e-6)
            check(link.inertial.mass > 0 and np.all(ev > 0) and tri,
                  f"{link.name}: m={link.inertial.mass*1000:.1f} g, inertia positive-definite & triangle inequality")

    print("4) loop closure  URDF tree  vs  kinematics")
    kin = Pm3rrrKinematics()
    rng = np.random.default_rng(0)
    errs, cerrs, n = [], [], 0
    while n < a.samples:
        pose = np.array([rng.uniform(-0.02, 0.02), rng.uniform(-0.02, 0.02), rng.uniform(-0.3, 0.3)])
        ik = kin.ik(pose)
        if not ik.ok:
            continue
        n += 1
        robot.update_cfg(kin.joint_state(pose, ik))
        T_pl = robot.get_transform("platform_link", "world")
        T_pc = robot.get_transform("platform_center", "world")
        cerrs.append(max(np.linalg.norm(T_pc[:2, 3] - pose[:2]),
                         abs(np.arctan2(T_pc[1, 0], T_pc[0, 0]) - pose[2])))
        for i in (2, 3):
            T_d = robot.get_transform(f"leg{i}_distal_link", "world")
            tip = T_d @ np.array([kin.p.l2, 0, 0, 1])                  # distal end in world
            ci = kin.c[i - 1] - kin.c[0]
            Ci = T_pl @ np.array([ci[0], ci[1], 0, 1])                 # platform hole in world
            errs.append(np.linalg.norm((tip - Ci)[:2]))
    check(max(cerrs) < 1e-6, f"platform_center equals commanded pose (max err {max(cerrs):.1e} m|rad)")
    check(max(errs) < 1e-6, f"legs 2/3 close on the platform over {n} poses (max gap {max(errs)*1e6:.3f} um)")
    print("\nRESULT:", "FAILED" if check.failed else "ALL CHECKS PASSED")
    sys.exit(1 if check.failed else 0)


if __name__ == "__main__":
    main()
