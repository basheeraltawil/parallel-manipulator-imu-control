"""pm3rrr_fake_hand_imu: a simulated hand-held MPU6050 (noise, bias, tremor).

Lets you try the IMU teleoperation without hardware. The "hand" repeats:
still (calibration) -> tilt forward -> tilt left -> circular stir -> rest.
Publishes sensor_msgs/Imu on /hand_imu/imu at 100 Hz and the true hand
orientation as a TF frame "hand_imu" (shown in RViz next to the robot).
"""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from pm3rrr_core.imu import Mpu6050Model


def hand(t):
    t = t % 14.0
    if t < 2.0:
        return 0.0, 0.0
    if t < 5.0:
        s = np.sin(np.pi * (t - 2.0) / 3.0)
        return 0.0, np.radians(25) * s
    if t < 8.0:
        s = np.sin(np.pi * (t - 5.0) / 3.0)
        return np.radians(-22) * s, 0.0
    if t < 12.0:
        u = (t - 8.0) / 4.0
        a = np.radians(20) * np.sin(np.pi * u)
        return a * np.sin(2 * np.pi * u), a * np.cos(2 * np.pi * u)
    return 0.0, 0.0


def quat(roll, pitch):
    cr, sr, cp, sp = np.cos(roll / 2), np.sin(roll / 2), np.cos(pitch / 2), np.sin(pitch / 2)
    return sr * cp, cr * sp, -sr * sp, cr * cp      # x, y, z, w (yaw = 0)


class FakeHandImu(Node):
    def __init__(self):
        super().__init__("pm3rrr_fake_hand_imu")
        self.imu = Mpu6050Model(seed=1)
        self.pub = self.create_publisher(Imu, "/hand_imu/imu", 50)
        self.tf = TransformBroadcaster(self)
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.prev = (0.0, 0.0)
        self.create_timer(0.01, self.tick)

    def tick(self):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9 - self.t0
        r, p = hand(t)
        rate = np.array([(r - self.prev[0]) / 0.01, (p - self.prev[1]) / 0.01, 0.0])
        self.prev = (r, p)
        tremor = np.radians(0.6) * np.sin(2 * np.pi * 9 * t)
        acc, gyr = self.imu.measure(r + tremor, p, rate)
        m = Imu()
        m.header.stamp = now.to_msg()
        m.header.frame_id = "hand_imu"
        m.orientation_covariance[0] = -1.0          # orientation not provided (like a raw MPU6050)
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = (float(v) for v in acc)
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = (float(v) for v in gyr)
        self.pub.publish(m)
        tf = TransformStamped()
        tf.header.stamp = m.header.stamp
        tf.header.frame_id, tf.child_frame_id = "world", "hand_imu"
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = 0.12, -0.2, 0.16
        (tf.transform.rotation.x, tf.transform.rotation.y,
         tf.transform.rotation.z, tf.transform.rotation.w) = quat(r, p)
        self.tf.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = FakeHandImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
