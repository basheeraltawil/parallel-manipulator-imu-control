# 02 · Bill of materials and component selection

Three tiers, using the **same kinematics, URDF and software**. Only the actuators and electronics change. Prices are indicative (2026, single units).

## Tier 1: teaching kit (the original design)

| # | Part | Qty | Spec | ≈ Price |
|---|---|---|---|---|
| 1 | TowerPro **MG996R** servo | 3 | 1.08 N m @ 6 V, 0.14 s/60°, metal gears | 3 × 6 € |
| 2 | Arduino **Uno/Nano** (or ESP32, Teensy) | 1 | firmware: 19 kB flash, 0.75 kB RAM | 5–25 € |
| 3 | **MPU6050** (GY-521) | 1–2 | hand IMU (0x68), optional base/platform IMU (0x69) | 2 × 3 € |
| 4 | 6 V / 5 A power supply (or 2S LiPo + 6 V UBEC) | 1 | servos stall at ~2.5 A each | 15 € |
| 5 | 624ZZ bearings (4 × 13 × 5) | 6 | platform and elbow joints | 6 × 1 € |
| 6 | M3/M4 screws, M3 self-locking nuts, brass bushings | set | see `hardware/cad/solidworks` | 5 € |
| 7 | PLA/PETG filament | ~150 g | 3D-printed parts (`hardware/stl`) | 4 € |
| 8 | Capacitor 1000 µF / 10 V, terminal block, wires, e-stop button | — | power hygiene, safety | 10 € |
| | **Total** | | | **≈ 90 €** |

## Tier 2: lab instrument (recommended upgrade)

Replace items 1–2 with **smart servos with encoders** and a USB adapter:

| Part | Why |
|---|---|
| 3 × Dynamixel **XL430-W250-T** (or Feetech **STS3215**) | 12-bit encoder (0.088°), position/velocity/current read-back, tiny dead band → S2 stop error **0.06 mm** |
| U2D2 / USB-TTL half-duplex adapter | PC talks to the servos directly; the Arduino becomes optional |
| BNO085 or ICM-42688-P IMU | on-chip fusion, lower drift than the MPU6050 |

Software impact: implement a `ros2_control` hardware interface for the servos (chapter 04). The URDF, controllers and maths stay the same.

## Tier 3: industrial stage

| Part | Example |
|---|---|
| Actuators | BLDC/stepper motors with 17-bit absolute encoders and zero-backlash reduction (harmonic, or direct drive); servo drives with STO (Safe Torque Off) |
| Joints | preloaded angular-contact bearings, hardened pins |
| Structure | machined aluminium 6061-T6 links, steel base plate |
| Controller | industrial PC or PLC with EtherCAT, ROS 2 on Linux RT |
| Sensing | encoders on the passive joints (detects assembly-mode problems), vision for calibration |
| Safety | certified e-stop chain, enabling device, guarding (chapter 07) |

## Selection rules used here

* **Torque:** inertial loads are tiny (lesson 08); size for process force + friction × safety factor 2, using *continuous* torque.
* **Speed:** platform speed ≈ motor speed × 50 mm/rad near home; the no-load speed limits throughput.
* **Resolution and dead band:** the Jacobian maps joint error to tool error (≈ 1.1 mm and 3° per degree, worst case). Choose the actuator from the accuracy target, not from torque.
* **IMU:** for human input any 6-axis IMU works. For stabilisation (S4), choose low gyro bias instability and a high ODR.

## Checklist

- [ ] Tier chosen and justified by the accuracy requirement
- [ ] Power supply sized for 3 × stall current
- [ ] Lead times checked for servos and bearings
