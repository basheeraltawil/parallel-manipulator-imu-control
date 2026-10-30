# 06 · Testing and validation

## Software tests (automated, CI)

| Layer | What | Where |
|---|---|---|
| Maths | IK/FK/Jacobian consistency, energy balance of the dynamics, trajectory limits, workspace, IMU filter, servo model | `ros2_ws/src/pm3rrr_core/test` (pytest) |
| Model | xacro expands, tree structure, meshes, inertia validity, **loop closure vs. maths** | `tools/cad_to_urdf/step4_validate_urdf.py`, `pm3rrr_description/test` |
| Nodes | scenario reachability, protocol parsing | `pm3rrr_control/test` |
| Firmware | C++ IK = Python IK, parser | `firmware/test` (g++) |
| System | the 5 scenarios produce their KPIs | `simulation/run_all.py --no-media` |
| ROS build | `colcon build` + `colcon test` in `ros:humble` | `.github/workflows/ros2.yml` |

## Hardware performance tests (ISO 9283 style)

ISO 9283 defines how to measure industrial-robot performance. A simplified version for this robot:

**Test poses:** P1 = centre, P2–P5 = the corners of a square inscribed in the usable circle (±14 mm), φ = 0.

| Characteristic | Procedure | Result |
|---|---|---|
| **Pose accuracy (AP)** | Approach each pose 30× from home; AP = distance between the mean reached pose and the commanded pose | mm |
| **Pose repeatability (RP)** | Same data: RP = $\bar l + 3 S_l$ (mean + 3σ of the distances to the barycentre) | mm |
| Orientation accuracy/repeatability | Same, for φ | ° |
| Path accuracy | Run the S3 contour, compare the measured path (camera) with the programmed one | mm |
| Cycle time | S2 sequence with a stopwatch or the log | s |
| Static stiffness | Hang known weights over a pulley on the platform, measure the deflection | N/mm |

**Measuring equipment** in increasing cost: calipers against hard stops, a dial gauge on a magnetic base, a top camera with an ArUco marker (OpenCV, ~0.05 mm at 30 px/mm), a laser tracker.

**Expected values** from the simulations: hobby servos AP ≈ 0.6 mm, smart servos ≈ 0.06 mm. If you measure much worse, look for joint play (chapter 03) and calibration (chapter 05).

## Endurance and environment
* 8-hour run of the figure-8 scenario: check servo temperature (< 60 °C), loosening screws, wear of the printed holes.
* Restart 50× (boot → calibrate → home) without the robot jumping.

## Checklist
- [ ] CI green
- [ ] AP/RP measured at 5 poses, 30 cycles each, and recorded
- [ ] Endurance run done
