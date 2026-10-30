# Hardware (original design files)

| Folder | Content | Used by |
|---|---|---|
| `cad/solidworks/` | SolidWorks parts and assemblies (`Assem1.SLDASM` = full robot) | design changes |
| `stl/` | 3D-printable parts, mm (exported from SolidWorks) | printing; `tools/cad_to_urdf` |
| `vendor_models/` | MG996R, MG90S and MG995/MG996 + bracket models (STEP/IGES/SLDPRT); `mg996r/MG996R_converted.stl` is the gmsh mesh used by the URDF | URDF visuals |
| `linkage/` | Linkage-2 sketches: `parallel_3rrr.linkage2` (concept of the 3-RRR), `serial_leg.linkage2` (one leg) | concept study |

How these files become the ROS robot model: [docs/learning/03_cad_to_urdf.md](../docs/learning/03_cad_to_urdf.md).
Printing, assembly and wiring: [docs/industrialization/03_mechanical_and_electrical.md](../docs/industrialization/03_mechanical_and_electrical.md).

> If you modify the CAD: export the changed part to `stl/` (mm), update `ros2_ws/src/pm3rrr_description/config/robot_params.yaml` if a dimension changed, then rerun `tools/cad_to_urdf/step2…step4`.
