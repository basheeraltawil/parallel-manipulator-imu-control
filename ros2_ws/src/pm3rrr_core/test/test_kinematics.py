import numpy as np
import pytest

from pm3rrr_core import DEFAULT_PARAMS, Pm3rrrKinematics, wrap
from pm3rrr_core.kinematics import rot2

kin = Pm3rrrKinematics()
rng = np.random.default_rng(1)


def random_poses(n=200, r=0.015, phi=0.25):
    out = []
    while len(out) < n:
        x = np.array([rng.uniform(-r, r), rng.uniform(-r, r), rng.uniform(-phi, phi)])
        if kin.ik(x).ok:
            out.append(x)
    return out


def test_home_is_symmetric_and_centred():
    ik = kin.ik((0.0, 0.0, 0.0))
    assert ik.ok
    # 120 deg symmetry of the design
    d = np.degrees(wrap(np.diff(ik.theta)))
    assert np.allclose(np.abs(d), 120.0, atol=1e-6)
    # servo centre = home pose -> joint values ~0
    assert np.allclose(kin.motor_to_joint(ik.theta), 0.0, atol=1e-3)


def test_links_have_correct_length():
    for x in random_poses(50):
        ik = kin.ik(x)
        assert np.allclose(np.linalg.norm(ik.elbows - kin.B, axis=1), DEFAULT_PARAMS.l1)
        assert np.allclose(np.linalg.norm(ik.platform_joints - ik.elbows, axis=1), DEFAULT_PARAMS.l2)


def test_report_equations_match_general_ik():
    """The report's a = d/2, h = sqrt(L^2 - a^2), alpha = atan2(h, a) (with the sign fixed)."""
    L = DEFAULT_PARAMS.l1
    x = (0.004, -0.006, 0.1)
    ik = kin.ik(x)
    d = ik.platform_joints - kin.B
    a = np.linalg.norm(d, axis=1) / 2
    h = np.sqrt(L**2 - a**2)
    alpha = np.arctan2(h, a)
    beta = np.arctan2(d[:, 1], d[:, 0])
    assert np.allclose(wrap(beta - alpha - ik.theta), 0.0, atol=1e-12)


def test_fk_inverts_ik():
    for x in random_poses(100):
        ik = kin.ik(x)
        guess = np.asarray(x) + np.array([0.002, -0.002, 0.03])
        sol = kin.fk(ik.theta, guess)
        assert sol is not None
        assert np.allclose(sol, x, atol=1e-9)


def test_fk_all_contains_true_pose():
    for x in random_poses(10):
        sols = kin.fk_all(kin.ik(x).theta)
        assert 1 <= len(sols) <= 6
        assert any(np.allclose(s, x, atol=1e-7) for s in sols)


def test_inverse_jacobian_matches_finite_difference():
    for x in random_poses(30):
        x = np.asarray(x)
        J = kin.inverse_jacobian(x)
        h = 1e-7
        Jfd = np.column_stack([
            wrap(kin.ik(x + h * e).theta - kin.ik(x - h * e).theta) / (2 * h) for e in np.eye(3)])
        assert np.allclose(J, Jfd, atol=1e-5)


def test_joint_state_reconstructs_platform_through_leg1():
    x = np.array([0.008, -0.004, 0.2])
    js = kin.joint_state(x)
    psi0, plat0 = kin.home_offsets
    th1 = kin.joint_to_motor([js[f"leg{i}_motor_joint"] for i in (1, 2, 3)])[0]
    a1 = th1 + psi0[0] + js["leg1_elbow_joint"]
    phi = a1 + plat0[0] + js["leg1_platform_joint"]
    A = kin.B[0] + DEFAULT_PARAMS.l1 * np.array([np.cos(th1), np.sin(th1)])
    C = A + DEFAULT_PARAMS.l2 * np.array([np.cos(a1), np.sin(a1)])
    centre = C - rot2(phi) @ kin.c[0]
    assert np.allclose(centre, x[:2], atol=1e-12)
    assert abs(wrap(phi - x[2])) < 1e-12


def test_unreachable_and_limit_flags():
    assert not kin.ik((0.2, 0.0, 0.0)).reachable
    res = kin.ik((0.0, 0.0, np.radians(80)))
    assert not res.ok


def test_servo_degree_mapping():
    assert kin.joint_to_servo_deg(0.0) == pytest.approx(90.0)
    assert kin.servo_deg_to_joint(180.0) == pytest.approx(np.pi / 2)


def test_yaml_matches_defaults():
    from pathlib import Path
    from pm3rrr_core import RobotParams
    yml = (Path(__file__).resolve().parents[2] / "pm3rrr_description" / "config" / "robot_params.yaml")
    if not yml.exists():
        pytest.skip("description package not next to core package")
    p = RobotParams.from_yaml(yml)
    d = DEFAULT_PARAMS
    for name in ("base_radius", "platform_radius", "l1", "l2", "joint_limit"):
        assert getattr(p, name) == pytest.approx(getattr(d, name))
    for name in ("base_angles", "platform_angles", "working_modes", "servo_center", "servo_direction"):
        assert np.allclose(getattr(p, name), getattr(d, name), atol=1e-4)


def test_home_is_all_zero_joints_and_matches_yaml():
    from pathlib import Path
    import yaml
    js = kin.joint_state((0.0, 0.0, 0.0), include_loop_closures=True)
    assert np.allclose(list(js.values()), 0.0, atol=1e-5)
    yml = (Path(__file__).resolve().parents[2] / "pm3rrr_description" / "config" / "robot_params.yaml")
    if not yml.exists():
        pytest.skip("description package not next to core package")
    g = yaml.safe_load(yml.read_text())["pm3rrr"]["geometry"]
    psi0, plat0 = kin.home_offsets
    assert np.degrees(psi0[0]) == pytest.approx(g["elbow_home_deg"], abs=1e-3)
    assert np.degrees(plat0[0]) == pytest.approx(g["platform1_home_deg"], abs=1e-3)
