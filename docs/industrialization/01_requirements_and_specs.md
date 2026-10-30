# 01 · Requirements and specification

Start with the **job**, not the robot. Fill in this table for your application; the values shown are for the teaching/lab version of this design. Each value is checked by a simulation or analysis in this repository.

| Requirement | Target (this design) | How it is verified |
|---|---|---|
| Degrees of freedom | 3 (x, y, φ), planar | lesson 01 |
| Usable task space | Ø 40 mm circle at φ = 0 | `largest_inscribed_circle`, lesson 07 |
| Orientation range (centre) | −22° … +50° (κ ≤ 10) | lesson 07 |
| Accuracy (absolute, after calibration) | hobby servos: ±0.8 mm · smart servos: ±0.1 mm | S2, accuracy map (lesson 11) |
| Repeatability | hobby: ±0.5 mm · smart: ±0.05 mm | test plan, chapter 06 |
| Max speed | 50 mm/s (controller limit; mechanism ~300 mm/s) | lesson 08, S3 |
| Contour accuracy at feed | ±0.5 mm at 40 mm/s | S3 |
| Payload | 200 g (horizontal) | lesson 08 |
| Push force | ≥ 10 N in any direction (centre) | `max_static_wrench`, lesson 08 |
| Interfaces | USB serial (ASCII protocol), ROS 2 Humble | chapter 04 |
| Operating modes | stream (PC), stand-alone IMU teleop, hold | firmware |
| Power | 6 V / 5 A servo supply + 5 V logic | chapter 03 |
| Safety | e-stop, singularity guard, watchdog, speed limit | chapter 07 |
| Environment | indoor, 10–35 °C, bench-top | — |

## Questions to answer before building

1. **Which accuracy do I really need?** It decides the actuator, the largest cost item (S2).
2. **Constant orientation or rotation too?** Rotation demands are limited by the singularity at −25° (S5).
3. **Who operates it?** Students: safety by low force and speed. Industry: a full risk assessment (chapter 07).
4. **How many units?** It decides printing vs. machining vs. moulding (chapter 08).

## Checklist

- [ ] Application and KPIs written down with numbers
- [ ] Workspace, accuracy and speed checked in simulation with *your* parameters
- [ ] Actuator tier chosen (chapter 02)
