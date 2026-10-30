"""pm3rrr_serial_bridge: ROS 2 <-> Arduino firmware (firmware/pm3rrr_firmware).

Line-based ASCII protocol, 115200 baud, every line ends with '\\n'
(documented in docs/industrialization/04_firmware_and_protocol.md):

  PC  -> MCU   J <s1> <s2> <s3>      servo angles [deg, 0..180, 2 decimals]
               P <x_mm> <y_mm> <phi_deg>   pose (IK on the microcontroller)
               H                     go home          M <0|1|2>  mode
               C                     calibrate gyro    ?          info
  MCU -> PC    I ax ay az gx gy gz   hand IMU [m/s^2, rad/s]  (100 Hz)
               S s1 s2 s3 mode       servo state [deg]       (10 Hz)
               E <text>              error / fault           OK <text>  ack

The firmware has a 0.5 s watchdog: if the PC stops sending, the robot holds.
"""

from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, String

try:
    import serial  # pyserial
except ImportError:  # pragma: no cover - only needed on the real robot
    serial = None


def parse_line(line: str):
    """Parse one telemetry line -> (kind, values or text). Pure function, unit-tested."""
    line = line.strip()
    if not line:
        return None, None
    kind, _, rest = line.partition(" ")
    if kind in ("I", "S"):
        try:
            return kind, [float(v) for v in rest.split()]
        except ValueError:
            return "E", f"malformed: {line}"
    if kind in ("E", "OK"):
        return kind, rest
    return "?", line


def format_servo_command(deg) -> str:
    return "J " + " ".join(f"{max(0.0, min(180.0, d)):.2f}" for d in deg) + "\n"


class SerialBridge(Node):
    def __init__(self):
        super().__init__("pm3rrr_serial_bridge")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        port = self.get_parameter("port").value
        if serial is None:
            raise RuntimeError("pyserial missing: sudo apt install python3-serial")
        self.ser = serial.Serial(port, int(self.get_parameter("baud").value), timeout=0.1)
        self.get_logger().info(f"connected to {port}")
        self.pub_imu = self.create_publisher(Imu, "/hand_imu/imu", 50)
        self.pub_state = self.create_publisher(Float64MultiArray, "/pm3rrr/servo_state_deg", 10)
        self.pub_status = self.create_publisher(String, "/pm3rrr/hardware_status", 10)
        self.create_subscription(Float64MultiArray, "/pm3rrr/servo_command_deg", self.on_cmd, 10)
        self.lock = threading.Lock()
        self.running = True
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()

    def on_cmd(self, msg: Float64MultiArray):
        with self.lock:
            self.ser.write(format_servo_command(msg.data).encode())

    def read_loop(self):
        while self.running and rclpy.ok():
            raw = self.ser.readline().decode(errors="ignore")
            kind, val = parse_line(raw)
            if kind == "I" and len(val) == 6:
                m = Imu()
                m.header.stamp = self.get_clock().now().to_msg()
                m.header.frame_id = "hand_imu"
                m.orientation_covariance[0] = -1.0
                (m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z,
                 m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z) = val
                self.pub_imu.publish(m)
            elif kind == "S":
                self.pub_state.publish(Float64MultiArray(data=val))
            elif kind in ("E", "OK", "?"):
                (self.get_logger().warn if kind == "E" else self.get_logger().info)(f"MCU: {val}")
                self.pub_status.publish(String(data=f"{kind} {val}"))

    def destroy_node(self):
        self.running = False
        try:
            self.ser.write(b"M 2\n")      # ask the firmware to hold
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
