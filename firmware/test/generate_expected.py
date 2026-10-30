#!/usr/bin/env python3
"""Reference IK values from pm3rrr_core for the firmware host test (CSV on stdout)."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "ros2_ws" / "src" / "pm3rrr_core"))
from pm3rrr_core import Pm3rrrKinematics  # noqa: E402

kin = Pm3rrrKinematics()
rng = np.random.default_rng(42)
print("x_mm,y_mm,phi_deg,s1,s2,s3,ok")
for _ in range(400):
    x = np.array([rng.uniform(-0.04, 0.04), rng.uniform(-0.04, 0.04), rng.uniform(-0.5, 0.4)])
    ik = kin.ik(x)
    m = kin.singularity_measures(x, ik) if ik.reachable else {"type1": 0, "type2": 0}
    sign = np.sign(np.linalg.det(kin.jacobians(x, ik)[0])) if ik.reachable else 0
    ok = ik.ok and m["type2"] * sign > 0.25 and m["type1"] > 0.30
    s = kin.joint_to_servo_deg(kin.motor_to_joint(ik.theta))
    print(f"{x[0]*1e3:.6f},{x[1]*1e3:.6f},{np.degrees(x[2]):.6f},{s[0]:.6f},{s[1]:.6f},{s[2]:.6f},{int(ok)}")
