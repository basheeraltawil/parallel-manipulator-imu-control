---
paths:
  - "**/*.py"
  - "firmware/**"
  - "**/*.xacro"
---
# Code style

## Python (pm3rrr_core, nodes, tools, simulation)
- Python ≥ 3.10, PEP 8, 4 spaces, max line length 110, `from __future__ import annotations`.
- numpy everywhere; vectorise when it stays readable. **SI units** in every function signature.
- Name maths variables after the docs: `B, A, C, c, theta, phi, sigma, Jx, Jq, kappa` (lesson notation, `docs/learning/00`).
- Each module starts with a docstring that says *what* it does and points to the lesson that explains *why*.
- `pm3rrr_core` must never import `rclpy` or other ROS packages.
- ROS nodes: one class per node, parameters declared in `__init__`, `main()` with try/finally shutdown.

## C++ (firmware)
- Arduino AVR: no `String`, no dynamic allocation, no `delay()` inside `loop()`.
- `float` with an `f` suffix; angles in radians internally, degrees only at the servo/protocol boundary.
- Pure logic (`kinematics.cpp`, `protocol.cpp`) must not include `Arduino.h`, so it stays host-testable.
- All tunables live in `config.h`.

## xacro / URDF
- Numbers come from `config/*.yaml` via `xacro.load_yaml`; no magic numbers in xacro.
- Keep the "zero = home" joint-origin convention.
