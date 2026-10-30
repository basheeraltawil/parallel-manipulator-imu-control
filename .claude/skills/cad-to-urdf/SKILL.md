---
name: cad-to-urdf
description: Regenerate the robot description (meshes, inertia, SRDF) after a CAD/STL or geometry change, and validate it. Use when STL files in hardware/stl, robot_params.yaml or the xacro files change, or the user asks to update the URDF/SRDF/meshes.
---
# CAD → URDF regeneration

Full explanation: `docs/learning/03_cad_to_urdf.md`.

1. If a dimension changed, update `ros2_ws/src/pm3rrr_description/config/robot_params.yaml` **and** `firmware/pm3rrr_firmware/config.h`. If the home pose changed, recompute `elbow_home_deg`/`platform1_home_deg` with `Pm3rrrKinematics().home_offsets`.
2. `python3 tools/cad_to_urdf/step1_inspect_parts.py --plot /tmp/parts.png`: check that the measured pivots match the YAML.
3. `python3 tools/cad_to_urdf/step2_build_meshes.py`: meshes + `inertials.yaml`. Then update the `RobotParams` inertia defaults if the masses changed.
4. `python3 tools/cad_to_urdf/step3_generate_srdf.py`: collision matrix.
5. `python3 tools/cad_to_urdf/step4_validate_urdf.py` must print **ALL CHECKS PASSED**.
6. `python3 -m pytest -q ros2_ws/src/pm3rrr_core/test && make -C firmware/test`.
7. Regenerate the media whose content changed: `python3 analysis/generate_figures.py render` and `python3 simulation/run_all.py`.
