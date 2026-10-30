"""pm3rrr_controller: platform pose in -> joint commands out.

The single place where Cartesian commands become motor commands, with the
safety logic an industrial controller needs:

  target_pose (geometry_msgs/Pose2D, metres/radians)
      -> rate limiter (max linear / angular speed)
      -> singularity guard (IK ok, servo limits, kappa, same assembly mode)
      -> IK -> outputs, depending on ``output_mode``:
           kinematic : /joint_states for every URDF tree joint (RViz-only sim)
           gazebo    : JointTrajectory to joint_trajectory_controller (physics)
           hardware  : servo angles in degrees for the serial bridge

Published diagnostics: /pm3rrr/commanded_pose, /pm3rrr/status (condition
number, fault text), /pm3rrr/markers (workspace outline, trail, target).
"""

from __future__ import annotations

from collections import deque

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose2D, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from pm3rrr_core import Pm3rrrKinematics, RobotParams
from pm3rrr_core.planning import assembly_sign

MOTORS = ["leg1_motor_joint", "leg2_motor_joint", "leg3_motor_joint"]
PLATFORM_TOP_Z = 0.0808 + 0.016 + 0.007


def yaw_to_quat(yaw: float):
    return 0.0, 0.0, float(np.sin(yaw / 2)), float(np.cos(yaw / 2))


class Pm3rrrController(Node):
    def __init__(self):
        super().__init__("pm3rrr_controller")
        self.declare_parameter("output_mode", "kinematic")       # kinematic | gazebo | hardware
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("max_linear_speed", 0.05)         # m/s
        self.declare_parameter("max_angular_speed", 1.0)         # rad/s
        self.declare_parameter("kappa_max", 10.0)
        self.declare_parameter("robot_params_file", "")          # optional robot_params.yaml
        self.declare_parameter("trajectory_lookahead", 0.06)     # s, gazebo mode

        f = self.get_parameter("robot_params_file").value
        params = RobotParams.from_yaml(f) if f else RobotParams()
        self.kin = Pm3rrrKinematics(params)
        self.mode = self.get_parameter("output_mode").value
        self.dt = 1.0 / float(self.get_parameter("rate_hz").value)
        self.vmax = float(self.get_parameter("max_linear_speed").value)
        self.wmax = float(self.get_parameter("max_angular_speed").value)
        self.kappa_max = float(self.get_parameter("kappa_max").value)
        self.lookahead = float(self.get_parameter("trajectory_lookahead").value)

        self.home_sign = assembly_sign(self.kin, (0.0, 0.0, 0.0))
        self.pose = np.zeros(3)            # last commanded (safe) pose
        self.goal = np.zeros(3)
        self.fault = ""
        self.trail = deque(maxlen=400)

        self.create_subscription(Pose2D, "/pm3rrr/target_pose", self.on_target, 10)
        self.pub_js = self.create_publisher(JointState, "/joint_states", 10) if self.mode == "kinematic" else None
        self.pub_traj = (self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)
                         if self.mode == "gazebo" else None)
        self.pub_servo = (self.create_publisher(Float64MultiArray, "/pm3rrr/servo_command_deg", 10)
                          if self.mode == "hardware" else None)
        self.pub_pose = self.create_publisher(PoseStamped, "/pm3rrr/commanded_pose", 10)
        self.pub_status = self.create_publisher(String, "/pm3rrr/status", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/pm3rrr/markers", 1)
        self.workspace_outline = self._workspace_outline()
        self.create_timer(self.dt, self.step)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info(f"pm3rrr_controller ready, output_mode={self.mode}")

    # ------------------------------------------------------------ callbacks
    def on_target(self, msg: Pose2D):
        self.goal = np.array([msg.x, msg.y, msg.theta])

    def safe(self, x):
        ik = self.kin.ik(x)
        if not ik.ok:
            return None, ik.message
        if self.kin.singularity_measures(x, ik)["kappa"] > self.kappa_max:
            return None, f"near singularity (kappa > {self.kappa_max:g})"
        if assembly_sign(self.kin, x) != self.home_sign:
            return None, "would change assembly mode"
        return ik, ""

    def step(self):
        # 1) rate limit toward the goal
        d = self.goal - self.pose
        n = np.linalg.norm(d[:2])
        if n > self.vmax * self.dt:
            d[:2] *= self.vmax * self.dt / n
        d[2] = np.clip(d[2], -self.wmax * self.dt, self.wmax * self.dt)
        cand = self.pose + d
        # 2) singularity guard: never step into an unsafe pose, hold instead
        ik, why = self.safe(cand)
        if ik is None:
            if why != self.fault:
                self.get_logger().warn(f"target rejected: {why}; holding last safe pose")
            self.fault = why
            ik = self.kin.ik(self.pose)
        else:
            self.fault = ""
            self.pose = cand
        self.trail.append(self.pose.copy())
        stamp = self.get_clock().now().to_msg()
        q = self.kin.motor_to_joint(ik.theta)

        # 3) outputs
        if self.pub_js is not None:
            js = self.kin.joint_state(self.pose, ik)
            m = JointState()
            m.header.stamp = stamp
            m.name = list(js.keys())
            m.position = list(js.values())
            self.pub_js.publish(m)
        if self.pub_traj is not None:
            t = JointTrajectory()
            t.joint_names = MOTORS
            p = JointTrajectoryPoint()
            p.positions = [float(v) for v in q]
            sec = self.lookahead
            p.time_from_start = Duration(sec=int(sec), nanosec=int((sec % 1) * 1e9))
            t.points = [p]
            self.pub_traj.publish(t)
        if self.pub_servo is not None:
            self.pub_servo.publish(Float64MultiArray(data=[float(v) for v in self.kin.joint_to_servo_deg(q)]))

        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = "world"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(self.pose[0]), float(self.pose[1]), PLATFORM_TOP_Z
        (ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w) = yaw_to_quat(self.pose[2])
        self.pub_pose.publish(ps)
        self.publish_markers(stamp)

    def publish_status(self):
        k = self.kin.singularity_measures(self.pose)["kappa"]
        txt = (f"pose=({self.pose[0]*1e3:.1f} mm, {self.pose[1]*1e3:.1f} mm, {np.degrees(self.pose[2]):.1f} deg) "
               f"kappa={k:.2f} " + (f"FAULT: {self.fault}" if self.fault else "OK"))
        self.pub_status.publish(String(data=txt))

    # -------------------------------------------------------------- markers
    def _workspace_outline(self, n=180):
        """Boundary of the usable workspace at phi = 0 (ray casting from the centre)."""
        pts = []
        for a in np.linspace(0, 2 * np.pi, n):
            r_ok = 0.0
            for r in np.linspace(0, 0.05, 101):
                x = (r * np.cos(a), r * np.sin(a), 0.0)
                if self.safe(x)[0] is None:
                    break
                r_ok = r
            pts.append((r_ok * np.cos(a), r_ok * np.sin(a)))
        return pts

    def publish_markers(self, stamp):
        arr = MarkerArray()

        def line(mid, pts, rgba, width, z=PLATFORM_TOP_Z):
            m = Marker()
            m.header.frame_id, m.header.stamp = "world", stamp
            m.ns, m.id, m.type, m.action = "pm3rrr", mid, Marker.LINE_STRIP, Marker.ADD
            m.scale.x = width
            m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
            m.pose.orientation.w = 1.0
            m.points = [Point(x=float(p[0]), y=float(p[1]), z=z) for p in pts]
            return m

        arr.markers.append(line(0, self.workspace_outline, (0.1, 0.7, 0.2, 0.8), 0.0008))
        if len(self.trail) > 1:
            arr.markers.append(line(1, list(self.trail), (1.0, 0.75, 0.0, 1.0), 0.0012, PLATFORM_TOP_Z + 0.001))
        g = Marker()
        g.header.frame_id, g.header.stamp = "world", stamp
        g.ns, g.id, g.type, g.action = "pm3rrr", 2, Marker.SPHERE, Marker.ADD
        g.pose.position.x, g.pose.position.y, g.pose.position.z = float(self.goal[0]), float(self.goal[1]), PLATFORM_TOP_Z + 0.002
        g.pose.orientation.w = 1.0
        g.scale.x = g.scale.y = g.scale.z = 0.003
        g.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.9) if self.fault else ColorRGBA(r=0.1, g=0.4, b=1.0, a=0.9)
        arr.markers.append(g)
        self.pub_markers.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = Pm3rrrController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
