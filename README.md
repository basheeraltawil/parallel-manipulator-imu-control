# Parallel Manipulator with IMU Control

A servo-driven **parallel manipulator** whose motion is controlled through **inverse kinematics** and monitored with an **MPU6050 accelerometer/gyroscope (IMU)**. The project includes the full mechanical design (SolidWorks CAD and 3D-printable STL files), Arduino firmware, and Python tools for kinematic analysis.

Developed as part of a Robotic Engineering Master's program (İzmir Katip Çelebi University).

<!-- TODO: add a photo or GIF of the robot here -->
<!-- ![Robot photo](docs/images/robot.jpg) -->

---

## Features

- Parallel manipulator driven by hobby servos (MG90S / MG996R)
- Inverse kinematics implemented in both **Python** (analysis) and **Arduino C++** (real-time)
- **MPU6050** accelerometer + gyroscope feedback for orientation sensing
- Full CAD model and 3D-printable parts
- Serial manipulator versions included for comparison with the parallel design

---

## Hardware

| Component | Details |
|---|---|
| Microcontroller | Arduino-compatible board (OpenCM documentation included in `docs/`) |
| Actuators | MG90S and MG996R / MG995 servo motors |
| Sensor | MPU6050 (accelerometer + gyroscope, I²C) |
| Structure | 3D-printed links, base, and end effector; M3 screws and nuts; brass bushings |
| Power | External power supply for the servos (do **not** power them from the Arduino 5V pin) |

<!-- TODO: add the exact degrees of freedom, board model, and servo count -->

### Wiring (typical)

| MPU6050 | Arduino (Uno/Nano) |
|---|---|
| VCC | 5V / 3.3V |
| GND | GND |
| SDA | A4 |
| SCL | A5 |

Servo signal wires connect to PWM-capable pins. Check the pin definitions at the top of each `.ino` file, since they may differ from the table above.

---

## Repository Structure

```
.
├── cad/                    SolidWorks parts and assemblies
│   └── vendor_models/      Downloaded servo models (MG90S, MG996R, MG995)
├── stl_files/              3D-printable STL parts
├── linkage/                Linkage mechanism simulations (.linkage2)
├── codes/                  Source code
│   ├── accelerometer/          Read raw accelerometer data
│   ├── aceleo_gyro/            Combined accelerometer + gyro reading
│   ├── gyro/servo_mpu6050/     Servo control driven by the MPU6050
│   ├── servo/                  Basic servo test
│   ├── first_manu/             First manipulator test
│   ├── singular_manipulators/  Serial manipulator versions (1, 2, 3)
│   ├── inverse_kin/            Inverse kinematics (Arduino)
│   ├── organised_inverse/      Cleaned-up inverse kinematics
│   ├── parallel_inverse_kinematics/   Parallel manipulator IK (Arduino)
│   ├── inverse kinematics.py   Inverse kinematics (Python)
│   ├── inverse_trying.py       IK experiments
│   └── parallel manipulator analyse.py   Workspace / kinematic analysis
├── inversetrying/          Early inverse kinematics sketch
└── docs/                   Report and presentations
```

---

## Getting Started

### 1. Clone the repository

```bash
git clone https://github.com/basheeraltawil/parallel-manipulator-imu-control.git
cd parallel-manipulator-imu-control
```

### 2. Run the Python analysis

```bash
pip install numpy matplotlib
python "codes/parallel manipulator analyse.py"
```

### 3. Upload the Arduino firmware

1. Install the [Arduino IDE](https://www.arduino.cc/en/software).
2. Install the required libraries via *Sketch → Include Library → Manage Libraries*:
   - `Servo` (built in)
   - `Wire` (built in)
   - `MPU6050` (e.g. by Electronic Cats or Jeff Rowberg)
3. Open the sketch you want, for example `codes/parallel_inverse_kinematics/parallel_inverse_kinematics.ino`.
4. Select your board and port, then click **Upload**.

### Suggested order for testing

1. `codes/servo` to check that the servos move
2. `codes/accelerometer` to check that the IMU reads correctly
3. `codes/parallel_inverse_kinematics` to run the manipulator
4. `codes/gyro/servo_mpu6050` to run servo control with IMU feedback

---

## Mechanical Design

- **CAD**: open `Assem1.SLDASM` in SolidWorks to see the full assembly.
- **3D printing**: use the files in `stl_files/`. Suggested settings: PLA, 0.2 mm layer height, 20–30% infill.
- **Fasteners**: M3 × 6, M3 × 10, and M3 self-locking nuts (models in `cad/`).

---

## Documentation

- `docs/report.pdf`: full project report
- `docs/Presentation1.pptx`, `docs/RO1.pptx`: presentation slides
- `docs/Open CM Microcontroller.docx`: microcontroller notes

---

## Roadmap

- [ ] Add photos and a demo video
- [ ] Add wiring diagram
- [ ] Add closed-loop control using IMU feedback
- [ ] Clean up and merge the experimental IK sketches

---

## Author

**Basheer Altawil**
GitHub: [@basheeraltawil](https://github.com/basheeraltawil)

## License

<!-- Choose a license, e.g. MIT, and add a LICENSE file -->
This project is released under the MIT License. See `LICENSE` for details.