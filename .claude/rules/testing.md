---
paths:
  - "ros2_ws/src/**"
  - "firmware/**"
  - "tools/**"
  - "simulation/**"
---
# Testing rules

- Every change to the maths needs a test in `ros2_ws/src/pm3rrr_core/test`. Prefer **property tests** (IK∘FK = identity, Jacobian = finite difference, power = dE/dt) over hard-coded numbers.
- Changing the URDF or `robot_params.yaml` → `python3 tools/cad_to_urdf/step4_validate_urdf.py` must print `ALL CHECKS PASSED`.
- Changing firmware kinematics, protocol or config → `make -C firmware/test` (compares C++ with Python on 400 poses).
- Scenario KPIs quoted in the docs must match `python3 simulation/run_all.py --no-media`. Update the docs if they change.
- ROS nodes: keep logic in pure functions (e.g. `parse_line`, `build`) and test those; node-level tests run in CI under `ros:humble`.
- Never weaken a tolerance to make a test pass without explaining why in the commit message.
