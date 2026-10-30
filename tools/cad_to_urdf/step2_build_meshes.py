#!/usr/bin/env python3
"""STEP 2 of the CAD -> URDF pipeline: meshes in link frames + mass properties.

For every URDF link we
  1. load the exported part (STL in mm, or STEP converted with gmsh),
  2. apply the rigid transform  T_link<-part  that puts the part's joint axis
     on the link origin and its "next joint" on the +x axis (values from step 1),
  3. scale mm -> m (URDF/ROS uses SI units, REP-103),
  4. write a *visual* mesh (full detail) and a *collision* mesh (convex hull:
     fast and robust for physics engines),
  5. compute mass, centre of mass and inertia tensor with an effective
     density (3D-printed PLA is NOT solid!) plus hardware point masses,
  6. write everything to pm3rrr_description/config/inertials.yaml which the
     xacro reads -> no hand-copied numbers.

Usage:  python3 tools/cad_to_urdf/step2_build_meshes.py
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import trimesh
import yaml

ROOT = Path(__file__).resolve().parents[2]
STL = ROOT / "hardware" / "stl"
VENDOR = ROOT / "hardware" / "vendor_models"
DESC = ROOT / "ros2_ws" / "src" / "pm3rrr_description"
MESH_V = DESC / "meshes" / "visual"
MESH_C = DESC / "meshes" / "collision"

PLA_DENSITY = 1240.0      # kg/m^3 solid PLA
FILL_FACTOR = 0.60        # thin parts printed with 3 perimeters + 25 % infill ~ 60 % solid
SERVO_MASS = 0.055        # MG996R datasheet: 55 g

# ---------------------------------------------------------------- transforms


def T(R=np.eye(3), t=(0, 0, 0)):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


def Rz(deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# part y-up -> z-up  (x, y, z) -> (x, -z, y)
Y_UP_TO_Z_UP = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)


def load_mg996r() -> trimesh.Trimesh:
    """Vendor STEP -> triangle mesh (gmsh/OpenCASCADE), cached next to the STEP."""
    cache = VENDOR / "mg996r" / "MG996R_converted.stl"
    if not cache.exists():
        import gmsh
        gmsh.initialize()
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.occ.importShapes(str(VENDOR / "mg996r" / "Servo MG996R.STEP"))
        gmsh.model.occ.synchronize()
        gmsh.option.setNumber("Mesh.MeshSizeMin", 1.5)
        gmsh.option.setNumber("Mesh.MeshSizeMax", 4.0)
        gmsh.model.mesh.generate(2)
        gmsh.write(str(cache))
        gmsh.finalize()
    return trimesh.load(cache)


# Every entry: link-frame transform applied to the part, in millimetres.
# The numbers come from step1 (hole / arc centres) and the stack-up in robot_params.yaml.
H = yaml.safe_load((DESC / "config" / "robot_params.yaml").read_text())["pm3rrr"]["heights_mm"]
PLATE, CRADLE_H = H["base_plate"], 45.0
SHAFT_OFFSET = 10.15           # MG996R: output shaft to body centre [mm]
FLANGE_BOTTOM = 25.0           # MG996R mesh: flange underside height [mm]
SHAFT_TOP = 47.6               # MG996R mesh: spline top [mm]
HORN_T = 5.2
HORN_TOP = PLATE + CRADLE_H + (SHAFT_TOP - FLANGE_BOTTOM) + HORN_T     # 80.8 mm

PARTS = {
    # name: (loader, transform part->link [mm], role)
    "base_plate": (lambda: trimesh.load(STL / "base.STL"),
                   T(t=(-100.0, -100.0, 0.0)), "base_link"),
    "servo_cradle": (lambda: trimesh.load(STL / "base22.STL"),
                     T(t=(-28.5 + SHAFT_OFFSET, -12.0, PLATE)), "servo frame S_i"),
    "mg996r": (load_mg996r,
               T(Y_UP_TO_Z_UP, (0, 0, 0)) @ T(t=(-10.0, -FLANGE_BOTTOM + PLATE + CRADLE_H, 10.0)),
               "servo frame S_i"),
    "servo_horn": (lambda: trimesh.load(STL / "servo dise v1.STL"),
                   T(Y_UP_TO_Z_UP) @ T(t=(-10.2, -40.8, -10.2)), "proximal link (below origin)"),
    "proximal_link": (lambda: trimesh.load(STL / "connecting_bar.STL"),
                      T(Rz(180)) @ T(t=(-58.35, -15.29, 0.0)), "proximal link"),
    "distal_link": (lambda: trimesh.load(STL / "middle_bar.STL"),
                    T(t=(-9.0, -9.0, 0.0)), "distal link"),
    "joint_pin": (lambda: trimesh.load(STL / "b_connection1.STL"),
                  T(t=(-7.5, -7.5, -11.0)), "distal link (elbow pin)"),
    "platform": (lambda: trimesh.load(STL / "end_effector.STL"),
                 T(Rz(60)) @ T(t=(-31.0, -21.49, 0.0)), "platform centre frame"),
}


def build():
    MESH_V.mkdir(parents=True, exist_ok=True)
    MESH_C.mkdir(parents=True, exist_ok=True)
    meshes = {}
    for name, (loader, M, role) in PARTS.items():
        m = loader()
        if isinstance(m, trimesh.Scene):
            m = m.dump(concatenate=True)
        m.apply_transform(M)
        m.apply_scale(0.001)                       # mm -> m
        m.export(MESH_V / f"{name}.stl")
        hull = m.convex_hull
        hull.export(MESH_C / f"{name}.stl")
        meshes[name] = m
        lo, hi = m.bounds * 1000
        print(f"{name:14s} -> {role:28s} faces={len(m.faces):6d} hull={len(hull.faces):4d} "
              f"bounds_mm=({lo.round(1)}, {hi.round(1)})")
    return meshes


def body_inertia(parts, point_masses=()):
    """Combine printed parts (effective density) and point masses into one rigid body."""
    mass, first, second = 0.0, np.zeros(3), np.zeros((3, 3))
    for m, density in parts:
        m = m.copy()
        m.density = density
        mass += m.mass
        first += m.mass * m.center_mass
        # inertia about the link origin = I_cm + m (|c|^2 I - c c^T)
        c = m.center_mass
        second += m.moment_inertia + m.mass * (c @ c * np.eye(3) - np.outer(c, c))
    for pm, pos in point_masses:
        pos = np.asarray(pos, float)
        mass += pm
        first += pm * pos
        second += pm * (pos @ pos * np.eye(3) - np.outer(pos, pos))
    com = first / mass
    I_cm = second - mass * (com @ com * np.eye(3) - np.outer(com, com))
    return mass, com, I_cm


def inertials(meshes):
    rho = PLA_DENSITY * FILL_FACTOR
    steel = 7850.0
    L = 0.050
    out = {}
    specs = {
        # horn screws (4 x M2 ~ 1 g) at the hub
        "proximal_link": ([(meshes["proximal_link"], rho), (meshes["servo_horn"], 1150.0)],
                          [(0.001, (0.0, 0.0, 0.004))]),
        # 624ZZ bearing (4x13x5, 2.6 g) at the platform end, steel pin at the elbow
        "distal_link": ([(meshes["distal_link"], rho), (meshes["joint_pin"], 1150.0)],
                        [(0.0026, (L, 0.0, 0.004))]),
        # 3 x M4 shoulder screws + nuts at the bearing seats (3 x 2 g)
        "platform": ([(meshes["platform"], rho)],
                     [(0.002, (0.026 * np.cos(a), 0.026 * np.sin(a), 0.0035))
                      for a in np.radians([30, 150, 270])]),
        "servo_assembly": ([(meshes["servo_cradle"], rho)], []),
        "base_plate": ([(meshes["base_plate"], rho)], []),
    }
    for name, (parts, pms) in specs.items():
        m, com, I = body_inertia(parts, pms)
        if name == "servo_assembly":      # servo body: datasheet mass, mesh geometry
            ms = meshes["mg996r"].copy()
            ms.density = SERVO_MASS / ms.volume
            m, com, I = body_inertia(parts + [(ms, ms.density)])
        out[name] = {"mass": round(float(m), 6), "com": [round(float(v), 6) for v in com],
                     "ixx": float(f"{I[0, 0]:.4e}"), "iyy": float(f"{I[1, 1]:.4e}"),
                     "izz": float(f"{I[2, 2]:.4e}"), "ixy": float(f"{I[0, 1]:.4e}"),
                     "ixz": float(f"{I[0, 2]:.4e}"), "iyz": float(f"{I[1, 2]:.4e}")}
        print(f"{name:15s} m={m*1000:6.1f} g  com={np.round(com*1000, 2)} mm  Izz={I[2, 2]:.3e} kg m^2")
    header = ("# GENERATED by tools/cad_to_urdf/step2_build_meshes.py - do not edit by hand.\n"
              f"# Effective density = {PLA_DENSITY} kg/m^3 x fill {FILL_FACTOR} (3D-printed PLA).\n"
              "# Units: kg, m, kg m^2. Inertia about the CoM, expressed in the link frame.\n")
    (DESC / "config" / "inertials.yaml").write_text(header + yaml.safe_dump(out, sort_keys=False))
    print(f"wrote {DESC / 'config' / 'inertials.yaml'}")
    print(f"servo horn top (motor joint height) = {HORN_TOP:.1f} mm")


if __name__ == "__main__":
    inertials(build())
