import numpy as np
import pytest

from pm3rrr_core import Pm3rrrDynamics, Pm3rrrKinematics
from pm3rrr_core import trajectory as tr
from pm3rrr_core.imu import ComplementaryFilter, Mpu6050Model, TiltTeleop
from pm3rrr_core.servo import ServoModel
from pm3rrr_core.workspace import constant_orientation, largest_inscribed_circle

kin = Pm3rrrKinematics()


@pytest.mark.parametrize("gravity", [(0.0, 0.0), (0.0, -9.81)])
def test_power_balance(gravity):
    """Motor power must equal the rate of change of total energy (validates inverse dynamics)."""
    dyn = Pm3rrrDynamics(kin, gravity=gravity)
    traj = tr.sample(tr.circle((0.0, 0.0), 0.01, phi=0.05), 1.0, dt=0.001, profile="quintic")
    E = np.array([dyn.kinetic_energy(x, xd) + dyn.potential_energy(x) for x, xd in zip(traj.x, traj.xd)])
    dEdt = np.gradient(E, traj.t)
    for k in range(50, len(traj) - 50, 97):
        x, xd, xdd = traj.x[k], traj.xd[k], traj.xdd[k]
        tau = dyn.inverse_dynamics(x, xd, xdd)
        thetad = kin.inverse_jacobian(x) @ xd
        assert tau @ thetad == pytest.approx(dEdt[k], rel=2e-3, abs=1e-7)


def test_static_wrench_consistent_with_virtual_work():
    dyn = Pm3rrrDynamics(kin)
    x = np.array([0.005, 0.003, 0.05])
    w = np.array([1.0, -0.5, 0.01])
    tau = dyn.static_torques(x, w)
    tau_id = dyn.inverse_dynamics(x, np.zeros(3), np.zeros(3), wrench=w)
    assert np.allclose(tau, tau_id)


@pytest.mark.parametrize("profile", ["cubic", "quintic", "trapezoidal"])
def test_scalings_boundary_conditions(profile):
    s, ds, dds = tr.SCALINGS[profile](np.array([0.0, 1.0]))
    assert np.allclose(s, [0, 1]) and np.allclose(ds, [0, 0])
    if profile == "quintic":
        assert np.allclose(dds, [0, 0])
    # integral of ds must be 1
    tau = np.linspace(0, 1, 20001)
    _, ds, _ = tr.SCALINGS[profile](tau)
    trapz = getattr(np, "trapezoid", None) or np.trapz
    assert trapz(ds, tau) == pytest.approx(1.0, abs=1e-4)


def test_min_duration_respects_limits():
    for prof in ("cubic", "quintic", "trapezoidal"):
        T = tr.min_duration(0.03, vmax=0.05, amax=0.3, profile=prof)
        traj = tr.sample(tr.line((0, 0, 0), (0.03, 0, 0)), T, dt=1e-4, profile=prof)
        assert np.abs(traj.xd[:, 0]).max() <= 0.05 * 1.001
        assert np.abs(traj.xdd[:, 0]).max() <= 0.3 * 1.01


def test_joint_space_mapping_ok_on_small_circle():
    traj = tr.sample(tr.circle((0, 0), 0.012), 2.0, dt=0.01)
    q, qd, qdd, ok = tr.to_joint_space(kin, traj)
    assert ok.all()
    assert np.abs(qd).max() < 2.0


def test_workspace_has_useful_size():
    w = constant_orientation(kin, 0.0, half_size=0.05, n=61)
    assert w.reachable[30, 30]
    r = largest_inscribed_circle(w, kappa_max=5.0)
    assert r > 0.015      # > 30 mm diameter usable circle at phi = 0


def test_complementary_filter_converges_and_rejects_bias():
    imu = Mpu6050Model(seed=3)
    cf = ComplementaryFilter(tau=0.5, dt=0.01)
    still = [imu.measure(0, 0, np.zeros(3))[1] for _ in range(200)]
    cf.calibrate(still)
    roll_true = np.radians(20)
    for _ in range(600):
        acc, gyr = imu.measure(roll_true, 0.0, np.zeros(3))
        r, p = cf.update(acc, gyr, 0.01)
    assert r == pytest.approx(roll_true, abs=np.radians(1.0))
    assert p == pytest.approx(0.0, abs=np.radians(1.0))


def test_teleop_rate_limit_and_reachability():
    tele = TiltTeleop(kin=kin, max_speed=0.05)
    pos = [tele.update(0.0, np.radians(30), 0.01)[0] for _ in range(10)]
    assert np.all(np.diff(pos) <= 0.05 * 0.01 + 1e-12)
    for _ in range(500):
        t = tele.update(np.radians(30), np.radians(30), 0.01)
    assert kin.ik(t).ok


def test_servo_quantisation():
    s = ServoModel()
    q = s.quantise(np.radians([0.4, 0.6, 1.4]))
    assert np.allclose(np.degrees(q), [0, 1, 1])
    s_us = ServoModel(use_microseconds=True)
    assert abs(np.degrees(s_us.quantise(np.radians(0.4))) - 0.4) < 0.1
