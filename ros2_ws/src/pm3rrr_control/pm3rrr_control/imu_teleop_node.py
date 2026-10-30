"""pm3rrr_imu_teleop: hand-held IMU tilt -> /pm3rrr/target_pose (scenario S1).

Input  : sensor_msgs/Imu on /hand_imu/imu (real MPU6050 via the serial bridge,
         or the simulated one from pm3rrr_fake_hand_imu)
Process: gyro-bias calibration (first ``calibration_time`` s, keep still!),
         complementary filter, dead band, expo, rate limit (pm3rrr_core.imu)
Output : geometry_msgs/Pose2D on /pm3rrr/target_pose
"""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from sensor_msgs.msg import Imu

from pm3rrr_core import Pm3rrrKinematics
from pm3rrr_core.imu import ComplementaryFilter, TiltTeleop


class ImuTeleop(Node):
    def __init__(self):
        super().__init__("pm3rrr_imu_teleop")
        self.declare_parameter("calibration_time", 1.0)
        self.declare_parameter("filter_tau", 0.4)
        self.declare_parameter("max_tilt_deg", 25.0)
        self.declare_parameter("deadband_deg", 3.0)
        self.declare_parameter("expo", 0.35)
        self.declare_parameter("reach", 0.016)
        self.declare_parameter("max_speed", 0.05)
        g = lambda n: float(self.get_parameter(n).value)  # noqa: E731
        self.cf = ComplementaryFilter(tau=g("filter_tau"), dt=0.01)
        self.tele = TiltTeleop(max_tilt=np.radians(g("max_tilt_deg")), deadband=np.radians(g("deadband_deg")),
                               expo=g("expo"), reach=g("reach"), max_speed=g("max_speed"), kin=Pm3rrrKinematics())
        self.calib_samples, self.calib_until, self.last = [], None, None
        self.pub = self.create_publisher(Pose2D, "/pm3rrr/target_pose", 10)
        self.create_subscription(Imu, "/hand_imu/imu", self.on_imu, 50)
        self.get_logger().info("hold the IMU still for calibration...")

    def on_imu(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyr = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        if self.calib_until is None:
            self.calib_until = t + float(self.get_parameter("calibration_time").value)
        if t < self.calib_until:
            self.calib_samples.append(gyr)
            return
        if self.calib_samples:
            self.cf.calibrate(self.calib_samples)
            self.get_logger().info(f"gyro bias calibrated: {np.degrees(self.cf.bias).round(2)} deg/s")
            self.calib_samples = []
        dt = 0.01 if self.last is None else max(1e-4, min(0.1, t - self.last))
        self.last = t
        self.cf.dt = dt
        roll, pitch = self.cf.update(acc, gyr, dt)
        x = self.tele.update(roll, pitch, dt)
        self.pub.publish(Pose2D(x=float(x[0]), y=float(x[1]), theta=float(x[2])))


def main(args=None):
    rclpy.init(args=args)
    node = ImuTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
