#include <math.h>
#include <stdint.h>

#include "config.h"
#include "kinematics.h"

#ifndef PI
#define PI 3.14159265358979f
#endif

static float Bx[3], By[3], cx[3], cy[3], center[3];

static float wrap(float a) {
  while (a > PI) a -= 2.0f * PI;
  while (a <= -PI) a += 2.0f * PI;
  return a;
}

void pm_kinematics_init() {
  for (int i = 0; i < 3; ++i) {
    float lb = PM_BASE_ANGLE_DEG[i] * PI / 180.0f;
    float lp = PM_PLATFORM_ANGLE_DEG[i] * PI / 180.0f;
    Bx[i] = PM_BASE_RADIUS * cosf(lb);
    By[i] = PM_BASE_RADIUS * sinf(lb);
    cx[i] = PM_PLATFORM_RADIUS * cosf(lp);
    cy[i] = PM_PLATFORM_RADIUS * sinf(lp);
    center[i] = SERVO_CENTER_DEG[i] * PI / 180.0f;
  }
}

IkResult pm_ik(const Pose &p) {
  IkResult r;
  r.reachable = true;
  r.within_limits = true;
  const float c = cosf(p.phi), s = sinf(p.phi);
  float ux[3], uy[3], rcx[3], rcy[3];
  r.type1 = 1e9f;
  for (int i = 0; i < 3; ++i) {
    // platform joint C_i = p + R(phi) c_i
    rcx[i] = c * cx[i] - s * cy[i];
    rcy[i] = s * cx[i] + c * cy[i];
    const float Cx = p.x + rcx[i], Cy = p.y + rcy[i];
    const float dx = Cx - Bx[i], dy = Cy - By[i];
    const float d = sqrtf(dx * dx + dy * dy);
    // law of cosines (report eq. 1-7, with the sign of eq. 4 corrected)
    float ca = (PM_L1 * PM_L1 + d * d - PM_L2 * PM_L2) / (2.0f * PM_L1 * d);
    if (ca > 1.0f || ca < -1.0f) {
      r.reachable = false;
      ca = ca > 1.0f ? 1.0f : -1.0f;
    }
    const float alpha = acosf(ca);
    const float beta = atan2f(dy, dx);
    r.theta[i] = wrap(beta + PM_WORKING_MODE[i] * alpha);
    const float q = wrap((r.theta[i] - center[i]) * SERVO_DIRECTION[i]);
    r.servo_deg[i] = 90.0f + q * 180.0f / PI;
    if (fabsf(q) > SERVO_LIMIT_DEG * PI / 180.0f) r.within_limits = false;
    // distal link vector u_i = C_i - A_i (for the Jacobian)
    const float Ax = Bx[i] + PM_L1 * cosf(r.theta[i]);
    const float Ay = By[i] + PM_L1 * sinf(r.theta[i]);
    ux[i] = Cx - Ax;
    uy[i] = Cy - Ay;
    // Jq_ii = u_i . E (A_i - B_i)   (serial-singularity measure)
    const float jq = fabsf(-ux[i] * (Ay - By[i]) + uy[i] * (Ax - Bx[i])) / (PM_L1 * PM_L2);
    if (jq < r.type1) r.type1 = jq;
  }
  // Jx rows: [u_x, u_y, u . (E R c)],  E R c = (-rcy, rcx)
  float J[3][3];
  for (int i = 0; i < 3; ++i) {
    J[i][0] = ux[i];
    J[i][1] = uy[i];
    J[i][2] = -ux[i] * rcy[i] + uy[i] * rcx[i];
  }
  const float det = J[0][0] * (J[1][1] * J[2][2] - J[1][2] * J[2][1]) -
                    J[0][1] * (J[1][0] * J[2][2] - J[1][2] * J[2][0]) +
                    J[0][2] * (J[1][0] * J[2][1] - J[1][1] * J[2][0]);
  r.type2 = det / (PM_L2 * PM_L2 * PM_L2 * PM_PLATFORM_RADIUS);   // signed: + = home assembly mode
  r.well_conditioned = !SINGULARITY_GUARD_ENABLED || (r.type2 > TYPE2_MIN && r.type1 > TYPE1_MIN);
  return r;
}
