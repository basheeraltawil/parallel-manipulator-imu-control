# Original documents (2020)

| File | Content |
|---|---|
| `report.pdf` / `report.docx` | "3 DoF Parallel robot manipulators with gyroscope trajectory generation": geometric IK, working modes |
| `ik_figures_presentation.pptx`, `ik_figures_RO1.pptx` | figures of the robot sketch and the two IK solutions $\theta=\beta\pm\alpha$ |
| `opencm_microcontroller_notes.docx` | notes on the ROBOTIS OpenCM 9.04 board and the Dynamixel XL-320 smart servo |

## Errata (corrected in the new learning material)

| Report | Correction | Where |
|---|---|---|
| Eq. (4): $h = \sqrt{L^2 + a^2}$ | $h = \sqrt{L^2 - a^2}$ (Pythagoras, $L$ is the hypotenuse) | [lesson 04](../learning/04_inverse_kinematics.md) |
| "nine working mode regions" | $2^3 = 8$ working modes (the report's own table has 8 rows) | lesson 04 |
| crossing between working modes: "mixed serial and parallel singularities" | changing a **working mode** crosses a *serial* (Type-I) singularity; changing an **assembly mode** crosses a *parallel* (Type-II) one | [lessons 05–06](../learning/06_jacobian_singularities.md) |

## A note on the OpenCM / Dynamixel document

The original notes already considered the Dynamixel XL-320 (digital, 0.29° resolution, TTL bus). The simulations in this repository show that this direction was right. Smart servos with position feedback improve the positioning accuracy about 10× over hobby servos ([S2](../scenarios/s2_precision_pick_place.md)), and they are the recommended tier-2 upgrade ([industrialization/02](../industrialization/02_bill_of_materials.md)).
