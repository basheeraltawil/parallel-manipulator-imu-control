"""pm3rrr_fk_monitor: measured joint angles -> measured platform pose.

Subscribes to /joint_states (Gazebo's joint_state_broadcaster or the real
robot) and solves the forward kinematics with Newton's method, seeded with
the previous solution so it stays in the current assembly mode. Publishes
/pm3rrr/measured_pose and the tracking error vs /pm3rrr/commanded_pose.
"""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from pm3rrr_core import Pm3rrrKinematics

MOTORS = ["leg1_motor_joint", "leg2_motor_joint", "leg3_motor_joint"]


class FkMonitor(Node):
    def __init__(self):
        super().__init__("pm3rrr_fk_monitor")
        self.kin = Pm3rrrKinematics()
        self.guess = np.zeros(3)
        self.cmd = None
        self.pub = self.create_publisher(PoseStamped, "/pm3rrr/measured_pose", 10)
        self.pub_err = self.create_publisher(Float64MultiArray, "/pm3rrr/tracking_error", 10)
        self.create_subscription(JointState, "/joint_states", self.on_js, 20)
        self.create_subscription(PoseStamped, "/pm3rrr/commanded_pose", self.on_cmd, 10)

    def on_cmd(self, msg):
        q = msg.pose.orientation
        self.cmd = np.array([msg.pose.position.x, msg.pose.position.y, 2 * np.arctan2(q.z, q.w)])

    def on_js(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        if not all(m in pos for m in MOTORS):
            return
        theta = self.kin.joint_to_motor(np.array([pos[m] for m in MOTORS]))
        sol = self.kin.fk(theta, self.guess)
        if sol is None:
            self.get_logger().warn("forward kinematics did not converge", throttle_duration_sec=2.0)
            return
        self.guess = sol
        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "world"
        out.pose.position.x, out.pose.position.y = float(sol[0]), float(sol[1])
        out.pose.orientation.z, out.pose.orientation.w = float(np.sin(sol[2] / 2)), float(np.cos(sol[2] / 2))
        self.pub.publish(out)
        if self.cmd is not None:
            e = sol - self.cmd
            self.pub_err.publish(Float64MultiArray(data=[float(np.hypot(e[0], e[1]) * 1e3), float(np.degrees(e[2]))]))


def main(args=None):
    rclpy.init(args=args)
    node = FkMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
