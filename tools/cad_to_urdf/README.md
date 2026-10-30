# CAD → URDF pipeline

Run in order (full explanation: [docs/learning/03_cad_to_urdf.md](../../docs/learning/03_cad_to_urdf.md)):

```bash
python3 tools/cad_to_urdf/step1_inspect_parts.py --plot parts.png   # measure pivots in the STL parts
python3 tools/cad_to_urdf/step2_build_meshes.py                     # link-frame meshes, collision hulls, inertials.yaml
python3 tools/cad_to_urdf/step3_generate_srdf.py                    # SRDF with a loop-aware collision matrix
python3 tools/cad_to_urdf/step4_validate_urdf.py                    # must print ALL CHECKS PASSED
```

`_ros_shim/` is a minimal stand-in for `ament_index_python`, so `xacro` can resolve `$(find pm3rrr_description)` without a ROS installation. When ROS 2 is sourced, the real package is used.
