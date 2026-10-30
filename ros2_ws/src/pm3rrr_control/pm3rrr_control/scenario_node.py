"""pm3rrr_scenario: streams the target poses of a scenario to /pm3rrr/target_pose.

The SAME trajectories as the desktop simulations (simulation/scenarios) are
built with pm3rrr_core, so RViz/Gazebo/hardware all run identical motions.

Parameters
  scenario : microplate | contour | figure8 | rotation_limit | home
  loop     : repeat forever (default true)
"""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node

from pm3rrr_core import trajectory as tr


def build(name: str) -> tr.Trajectory:
    if name == "microplate":       # S2: 9 mm pitch wells, rest-to-rest, dwell
        p = 0.009
        wells = [(-p, p / 2), (0, p / 2), (p, p / 2), (p, -p / 2), (0, -p / 2), (-p, -p / 2)]
        pts = [(0, 0, 0)] + [(x, y, 0.0) for x, y in wells] + [(0, 0, 0)]
        return tr.point_to_point(pts, vmax=0.06, amax=0.6, dt=0.02, profile="quintic", dwell=0.5)
    if name == "contour":          # S3: rounded rectangle at 40 mm/s
        w, h, r = 0.028, 0.018, 0.004
        cx, cy = w / 2 - r, h / 2 - r
        pts = [np.array([[0.0, -h / 2]])]
        for ox, oy, a0 in ((cx, -cy, -90), (cx, cy, 0), (-cx, cy, 90), (-cx, -cy, 180)):
            a = np.radians(np.linspace(a0, a0 + 90, 20))
            pts.append(np.column_stack((ox + r * np.cos(a), oy + r * np.sin(a))))
        pts.append(np.array([[0.0, -h / 2]]))
        P = np.vstack(pts)
        s = np.concatenate(([0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))))
        T = s[-1] / 0.040 / 0.92
        t = np.arange(0, T, 0.02)
        u, _, _ = tr.trapezoidal(t / T, 0.08)
        xy = np.column_stack((np.interp(u * s[-1], s, P[:, 0]), np.interp(u * s[-1], s, P[:, 1])))
        lead = tr.point_to_point([(0, 0, 0), (0, -h / 2, 0)], dt=0.02)
        body = tr.Trajectory(t, np.column_stack((xy, np.zeros(len(t)))), np.zeros((len(t), 3)), np.zeros((len(t), 3)))
        back = tr.point_to_point([(0, -h / 2, 0), (0, 0, 0)], dt=0.02)
        return tr.Trajectory.concatenate([lead, body, back])
    if name == "figure8":
        return tr.sample(tr.lemniscate((0, 0), 0.016, phi_amp=np.radians(10)), 6.0, 0.02, "quintic")
    if name == "rotation_limit":   # S5: ask for -40 deg; the controller's guard stops near -22 deg
        return tr.point_to_point([(0, 0, 0), (0, 0, np.radians(-40)), (0, 0, np.radians(-40)), (0, 0, 0)],
                                 wmax=0.4, dt=0.02, dwell=1.0)
    return tr.point_to_point([(0, 0, 0), (0, 0, 0)], dt=0.02)


class ScenarioNode(Node):
    def __init__(self):
        super().__init__("pm3rrr_scenario")
        self.declare_parameter("scenario", "figure8")
        self.declare_parameter("loop", True)
        self.declare_parameter("start_delay", 2.0)
        self.name = self.get_parameter("scenario").value
        self.loop = bool(self.get_parameter("loop").value)
        self.traj = build(self.name)
        self.pub = self.create_publisher(Pose2D, "/pm3rrr/target_pose", 10)
        self.t0 = self.get_clock().now().nanoseconds * 1e-9 + float(self.get_parameter("start_delay").value)
        self.create_timer(0.02, self.tick)
        self.get_logger().info(f"scenario '{self.name}': {self.traj.t[-1]:.1f} s, loop={self.loop}")

    def tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        if t < 0:
            return
        T = self.traj.t[-1]
        if self.loop:
            t = t % (T + 1.0)
        k = min(int(round(t / 0.02)), len(self.traj) - 1)
        x = self.traj.x[k]
        self.pub.publish(Pose2D(x=float(x[0]), y=float(x[1]), theta=float(x[2])))


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
