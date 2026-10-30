// Closed-form inverse kinematics of the planar 3-RRR robot (no Arduino deps,
// unit-tested on the PC: firmware/test). Units: mm, rad.
#pragma once

struct Pose {
  float x, y, phi;
};

struct IkResult {
  float theta[3];        // world angle of each motor link [rad]
  float servo_deg[3];    // command for each servo [deg], 90 = centre
  bool reachable;        // every leg closes
  bool within_limits;    // every servo inside +-SERVO_LIMIT_DEG
  bool well_conditioned; // away from a parallel (Type-II) singularity
  float type1;           // min normalised |Jq_ii|  (0 when a leg is stretched/folded)
  float type2;           // normalised det Jx, signed (0 at a parallel singularity, <0 other assembly mode)
  bool ok() const { return reachable && within_limits && well_conditioned; }
};

void pm_kinematics_init();
IkResult pm_ik(const Pose &p);
