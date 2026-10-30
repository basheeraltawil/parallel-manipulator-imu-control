"""Physics simulation of the 3-RRR robot in MuJoCo, built from the SAME URDF used by ROS.

Why MuJoCo here? It runs anywhere with ``pip install mujoco`` (no ROS needed),
handles closed kinematic chains with equality constraints and renders
off-screen, so every student can reproduce the scenario videos. The ROS 2 /
Gazebo pipeline uses the identical URDF (see ros2_ws/src/pm3rrr_gazebo).

Pipeline:  xacro -> URDF -> MjSpec (+ loop closures, servos, scene) -> MjModel
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "pm3rrr_core"))
sys.path.insert(0, str(ROOT / "tools" / "cad_to_urdf"))
os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco  # noqa: E402

from pm3rrr_core import DEFAULT_PARAMS, Pm3rrrKinematics  # noqa: E402

COLORS = {
    "base_plate": (0.22, 0.24, 0.28, 1), "servo_cradle": (0.90, 0.50, 0.12, 1),
    "mg996r": (0.06, 0.06, 0.07, 1), "servo_horn": (0.92, 0.92, 0.92, 1),
    "proximal_link": (0.12, 0.45, 0.85, 1), "distal_link": (0.93, 0.93, 0.95, 1),
    "joint_pin": (0.70, 0.70, 0.72, 1), "platform": (0.88, 0.16, 0.20, 1),
}


def build_urdf() -> str:
    from step4_validate_urdf import expand_xacro, resolve_package_uris
    return resolve_package_uris(expand_xacro())


class MujocoPm3rrr:
    """Closed-chain physics model with hobby-servo-like position actuators.

    Servo model: torque = clip(kp (q_cmd - q) - kv qdot, +-stall_torque),
    plus reflected rotor inertia (armature). This is what the MG996R's
    internal potentiometer + H-bridge loop does.
    """

    def __init__(self, kp: float = 2.0, kv: float = 0.03, armature: float = 2e-5,
                 timestep: float = 0.0005, width: int = 640, height: int = 480, kin=None,
                 hand_imu: bool = False):
        self.kin = kin or Pm3rrrKinematics(DEFAULT_PARAMS)
        p = self.kin.p
        urdf = build_urdf().replace(
            '<robot name="pm3rrr">',
            '<robot name="pm3rrr"><mujoco><compiler discardvisual="false" strippath="false" '
            'fusestatic="true" balanceinertia="true"/></mujoco>', 1)
        spec = mujoco.MjSpec.from_string(urdf)
        spec.option.timestep = timestep
        spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        spec.option.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT   # no self-contact (proved by SRDF step)
        spec.visual.global_.offwidth = width
        spec.visual.global_.offheight = height

        # --- appearance: colour visual meshes, hide collision meshes
        mesh_file = {m.name: m.file for m in spec.meshes}
        for g in spec.geoms:
            f = mesh_file.get(g.meshname, "")
            if "/collision/" in f:
                g.group, g.contype, g.conaffinity = 3, 0, 0
                g.rgba = (0, 0, 0, 0)
            else:
                g.rgba = COLORS.get(Path(f).stem, (0.6, 0.6, 0.6, 1))

        # --- loop closures: legs 2 and 3 pin onto the platform.
        # Site-to-site "connect": the distal-link tip must coincide with the
        # platform bearing seat (same idea as the Gazebo loop-closure joint).
        lt = 0.008
        for i in (2, 3):
            tip = spec.body(f"leg{i}_distal_link").add_site()
            tip.name, tip.pos = f"leg{i}_tip", (p.l2, 0.0, lt)
            seat = spec.body("platform_link").add_site()
            ci = self.kin.c[i - 1] - self.kin.c[0]
            seat.name, seat.pos = f"platform_seat{i}", (ci[0], ci[1], 0.0)
            eq = spec.add_equality()
            eq.type = mujoco.mjtEq.mjEQ_CONNECT
            eq.objtype = mujoco.mjtObj.mjOBJ_SITE
            eq.name1, eq.name2 = tip.name, seat.name
            eq.solref = (0.002, 1.0)

        # --- servos
        for i in (1, 2, 3):
            j = spec.joint(f"leg{i}_motor_joint")
            j.armature = armature
            a = spec.add_actuator()
            a.name = f"servo{i}"
            a.target = f"leg{i}_motor_joint"
            a.trntype = mujoco.mjtTrn.mjTRN_JOINT
            a.set_to_position(kp=kp, kv=kv)
            a.forcelimited = True
            a.forcerange = (-p.servo.stall_torque, p.servo.stall_torque)

        # --- scene: sky, table, lights, platform-centre site
        sky = spec.add_texture()
        sky.name, sky.type, sky.builtin = "sky", mujoco.mjtTexture.mjTEXTURE_SKYBOX, mujoco.mjtBuiltin.mjBUILTIN_GRADIENT
        sky.rgb1, sky.rgb2, sky.width, sky.height = (0.93, 0.95, 0.98), (0.55, 0.62, 0.72), 256, 256
        wb = spec.worldbody
        floor = wb.add_geom()
        floor.type = mujoco.mjtGeom.mjGEOM_PLANE
        floor.size = (2.0, 2.0, 0.01)
        floor.pos = (0, 0, -0.0005)
        floor.rgba = (0.80, 0.78, 0.74, 1)
        for pos, dif in (((0.3, -0.3, 0.6), 0.6), ((-0.3, 0.2, 0.5), 0.35)):
            li = wb.add_light()
            li.pos, li.dir = pos, tuple(-np.asarray(pos) / np.linalg.norm(pos))
            li.diffuse = (dif, dif, dif)
        c1 = self.kin.c[0]
        site = spec.body("platform_link").add_site()
        site.name = "platform_center"
        site.pos = (-c1[0], -c1[1], 0.0075)
        site.size = (0.002, 0.002, 0.002)
        site.rgba = (1, 1, 0, 1)
        if hand_imu:   # a GY-521 board held "in the hand" (mocap = moved by us, not by physics)
            hb = wb.add_body()
            hb.name, hb.mocap, hb.pos = "hand_imu", True, (0.12, -0.20, 0.16)
            for size, pos, rgba in (((0.030, 0.020, 0.0016), (0, 0, 0), (0.10, 0.30, 0.75, 1)),
                                    ((0.006, 0.006, 0.0015), (0, 0, 0.003), (0.1, 0.1, 0.1, 1)),
                                    ((0.003, 0.010, 0.0025), (-0.026, 0, 0.004), (0.85, 0.75, 0.2, 1))):
                g = hb.add_geom()
                g.type, g.size, g.pos, g.rgba = mujoco.mjtGeom.mjGEOM_BOX, size, pos, rgba
                g.contype = g.conaffinity = 0
        self.spec = spec
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        self._renderer = None
        self.width, self.height = width, height
        m = self.model
        self.motor_qadr = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"leg{i}_motor_joint")]
                           for i in (1, 2, 3)]
        self.motor_dadr = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"leg{i}_motor_joint")]
                           for i in (1, 2, 3)]
        self.site_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "platform_center")

    # ---------------------------------------------------------------- state
    def set_pose(self, pose):
        """Assemble the robot exactly at a platform pose (all joints from the IK)."""
        js = self.kin.joint_state(pose)
        for name, val in js.items():
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            self.data.qpos[self.model.jnt_qposadr[jid]] = val
        self.data.qvel[:] = 0
        self.data.ctrl[:] = [js[f"leg{i}_motor_joint"] for i in (1, 2, 3)]
        mujoco.mj_forward(self.model, self.data)

    def set_hand(self, roll: float, pitch: float, yaw: float = 0.0):
        """Orient the floating hand-held IMU board (scenario 1)."""
        cr, sr, cp, sp, cy, sy = np.cos(roll / 2), np.sin(roll / 2), np.cos(pitch / 2), np.sin(pitch / 2), \
            np.cos(yaw / 2), np.sin(yaw / 2)
        self.data.mocap_quat[0] = [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                                   cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]

    def command(self, q_motor):
        self.data.ctrl[:] = q_motor

    def step(self, duration: float):
        n = max(1, int(round(duration / self.model.opt.timestep)))
        for _ in range(n):
            mujoco.mj_step(self.model, self.data)

    def motor_positions(self):
        return self.data.qpos[self.motor_qadr].copy()

    def motor_torques(self):
        return self.data.actuator_force.copy()

    def platform_pose(self):
        pos = self.data.site_xpos[self.site_id]
        R = self.data.site_xmat[self.site_id].reshape(3, 3)
        return np.array([pos[0], pos[1], np.arctan2(R[1, 0], R[0, 0])])

    def constraint_error(self):
        """Max loop-closure violation [m] (should stay ~micrometres)."""
        return float(np.max(np.abs(self.data.efc_pos[self.data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY]))
                     if self.data.nefc else 0.0)

    # --------------------------------------------------------------- render
    def render(self, azimuth=135.0, elevation=-38.0, distance=0.42, lookat=(0, 0, 0.06), trail=None,
               markers=(), trail_rgba=(1.0, 0.85, 0.1, 1.0)):
        """Off-screen render. ``trail``: list of xy points drawn as a line at platform height;
        ``markers``: list of (xyz, rgba, radius) spheres (targets, reference points)."""
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, self.height, self.width)
        cam = mujoco.MjvCamera()
        cam.azimuth, cam.elevation, cam.distance = azimuth, elevation, distance
        cam.lookat[:] = lookat
        self._renderer.update_scene(self.data, cam)
        if trail is not None and len(trail) > 1:
            scn = self._renderer.scene
            z = self.data.site_xpos[self.site_id][2] + 0.001
            for a, b in zip(trail[:-1], trail[1:]):
                if scn.ngeom >= scn.maxgeom:
                    break
                g = scn.geoms[scn.ngeom]
                mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3),
                                    np.zeros(9), np.array(trail_rgba, np.float32))
                mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.0012,
                                     np.array([a[0], a[1], z]), np.array([b[0], b[1], z]))
                scn.ngeom += 1
        scn = self._renderer.scene
        for xyz, rgba, rad in markers:
            if scn.ngeom >= scn.maxgeom:
                break
            mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, np.array([rad, rad, rad]),
                                np.asarray(xyz, float), np.eye(3).ravel(), np.array(rgba, np.float32))
            scn.ngeom += 1
        return self._renderer.render()
