"""Robot parameters for the planar 3-RRR parallel manipulator.

All values are SI (metres, radians, kilograms). The geometric values were
measured from the original SolidWorks/STL parts (see
``docs/learning/03_cad_to_urdf.md``):

* base pivots (servo axes) lie on a circle of radius 87.5 mm  (base.STL)
* platform joints lie on a circle of radius 26 mm             (end_effector.STL)
* both links measure 50 mm hole-to-hole                        (connecting_bar / middle_bar)

The same numbers live in ``pm3rrr_description/config/robot_params.yaml`` so
that the URDF and the maths can never drift apart (a unit test checks this).
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from math import radians
from pathlib import Path
from typing import Sequence, Tuple

import numpy as np

Vec3 = Tuple[float, float, float]


@dataclass(frozen=True)
class LinkInertia:
    """Planar rigid-body inertia of one body (about its own centre of mass)."""

    mass: float           # kg
    com: float            # distance of the CoM from the proximal pivot along the link [m]
    izz: float            # inertia about the CoM, z-axis [kg m^2]


@dataclass(frozen=True)
class ServoSpec:
    """Hobby-servo characteristics (default: TowerPro MG996R at 6 V)."""

    stall_torque: float = 1.08        # N m   (11 kgf cm @ 6 V, datasheet)
    no_load_speed: float = 7.0        # rad/s (0.14 s / 60 deg @ 6 V)
    resolution: float = radians(1.0)  # Servo.write() integer-degree quantisation
    deadband: float = radians(0.5)    # typical analogue servo dead band
    time_constant: float = 0.04       # s, first-order lag of the internal loop
    range: float = radians(180.0)     # usable travel


@dataclass(frozen=True)
class RobotParams:
    """Geometric, inertial and actuator parameters of the 3-RRR robot."""

    base_radius: float = 0.0875
    platform_radius: float = 0.026
    l1: float = 0.050                                   # proximal (motor) link
    l2: float = 0.050                                   # distal link
    base_angles: Vec3 = (radians(-30.0), radians(90.0), radians(210.0))
    platform_angles: Vec3 = (radians(30.0), radians(150.0), radians(270.0))
    working_modes: Vec3 = (-1.0, -1.0, -1.0)            # sigma_i: theta = beta + sigma*alpha
    # Servo mounting: world angle of the motor link when the servo is at its
    # centre (90 deg command). Chosen = home-pose angle so travel is symmetric.
    servo_center: Vec3 = (radians(94.286907), radians(-145.713093), radians(-25.713093))
    servo_direction: Vec3 = (1.0, 1.0, 1.0)             # +1: CCW seen from above
    joint_limit: float = radians(85.0)                  # +- around servo centre (keep 5 deg margin)

    # Inertial data (computed from the STL meshes, PLA, see tools/cad_to_urdf)
    # (values = pm3rrr_description/config/inertials.yaml; proximal includes the servo horn)
    proximal: LinkInertia = LinkInertia(mass=0.0063, com=0.0179, izz=2.77e-6)
    distal: LinkInertia = LinkInertia(mass=0.0080, com=0.0287, izz=3.89e-6)
    platform_mass: float = 0.0112
    platform_izz: float = 5.89e-6
    rotor_inertia: float = 2.0e-6                       # reflected servo rotor + horn [kg m^2]
    servo: ServoSpec = field(default_factory=ServoSpec)

    # ---------------------------------------------------------------- helpers
    def base_points(self) -> np.ndarray:
        """3x2 array: world coordinates of the motor axes B_i."""
        a = np.asarray(self.base_angles)
        return self.base_radius * np.column_stack((np.cos(a), np.sin(a)))

    def platform_points(self) -> np.ndarray:
        """3x2 array: platform joint positions c_i in the platform frame."""
        g = np.asarray(self.platform_angles)
        return self.platform_radius * np.column_stack((np.cos(g), np.sin(g)))

    def with_modes(self, modes: Sequence[float]) -> "RobotParams":
        return replace(self, working_modes=tuple(float(m) for m in modes))

    # ------------------------------------------------------------------- YAML
    @classmethod
    def from_yaml(cls, path: str | Path) -> "RobotParams":
        """Load the geometry from ``robot_params.yaml`` (degrees + millimetres)."""
        import yaml

        data = yaml.safe_load(Path(path).read_text())["pm3rrr"]
        g = data["geometry"]
        s = data["servo"]
        return cls(
            base_radius=g["base_radius_mm"] / 1000.0,
            platform_radius=g["platform_radius_mm"] / 1000.0,
            l1=g["proximal_link_mm"] / 1000.0,
            l2=g["distal_link_mm"] / 1000.0,
            base_angles=tuple(radians(v) for v in g["base_angles_deg"]),
            platform_angles=tuple(radians(v) for v in g["platform_angles_deg"]),
            working_modes=tuple(float(v) for v in g["working_modes"]),
            servo_center=tuple(radians(v) for v in s["center_deg"]),
            servo_direction=tuple(float(v) for v in s["direction"]),
            joint_limit=radians(s["limit_deg"]),
        )


DEFAULT_PARAMS = RobotParams()
