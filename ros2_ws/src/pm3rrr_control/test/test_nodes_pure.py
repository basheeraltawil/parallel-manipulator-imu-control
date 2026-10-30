"""Tests of the ROS-independent parts of the nodes (run without a ROS graph)."""
import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")  # the node modules import rclpy at module level

from pm3rrr_control.scenario_node import build  # noqa: E402
from pm3rrr_control.serial_bridge_node import format_servo_command, parse_line  # noqa: E402
from pm3rrr_core import Pm3rrrKinematics  # noqa: E402


@pytest.mark.parametrize("name", ["microplate", "contour", "figure8", "home"])
def test_scenarios_are_reachable(name):
    kin = Pm3rrrKinematics()
    tr = build(name)
    assert all(kin.ik(x).ok for x in tr.x)


def test_rotation_limit_scenario_needs_the_guard():
    kin = Pm3rrrKinematics()
    tr = build("rotation_limit")
    assert not all(kin.ik(x).ok and kin.singularity_measures(x)["kappa"] < 10 for x in tr.x)


def test_protocol_roundtrip():
    assert format_servo_command([90, 200, -5]) == "J 90.00 180.00 0.00\n"
    k, v = parse_line("I 0.1 -0.2 9.8 0.01 0.02 0.03\n")
    assert k == "I" and np.allclose(v, [0.1, -0.2, 9.8, 0.01, 0.02, 0.03])
    assert parse_line("E watchdog")[0] == "E"
    assert parse_line("S 90 90 x")[0] == "E"
