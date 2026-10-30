# 03 · Mechanical and electrical build

## Mechanical

### Printing (tier 1)

| Part (`hardware/stl/`) | Qty | Material | Settings / notes |
|---|---|---|---|
| `base.STL` | 1 | PETG/PLA | 0.2 mm layers, 4 perimeters, 30 % infill; flatness matters, print on glass |
| `base22.STL` (servo cradle) | 3 | PETG | servo fits between the uprights, flange on the ears |
| `connecting_bar.STL` (proximal) | 3 | PETG | print **flat**; the horn screw holes need ±0.1 mm, drill to size |
| `middle_bar.STL` (distal) | 3 | PETG | Ø13 bearing seat: print 12.9 mm, ream or press-fit the 624ZZ |
| `end_effector.STL` (platform) | 1 | PETG | 3 bearing seats on Ø52 mm; check with calipers |
| `b_connection1.STL` (pin) | 3 | PETG or turned steel/brass | better: M4 shoulder screw |
| `CIRCLE_SUP.STL` (spacer) | 6 | PLA | bearing spacers |

The simulations assumed ~60 % solid parts (3 perimeters + 25 % infill), so keep these settings or regenerate `inertials.yaml`.

### Assembly order

1. Press the bearings into the distal links and the platform.
2. Mount the servos in the cradles, then the cradles on the base (M3).
3. **Centre the servos electrically first** (`firmware/bringup/servo_calibration`, 1500 µs), *then* fit the horns and proximal links as close to the home direction (lesson 02 table) as the spline allows. The remainder is the trim, measured in chapter 05.
4. Fit the elbow pins, distal links, then the platform last.

### Tolerances and play: where accuracy is lost

* Each revolute joint with 0.05 mm radial play adds up to 0.05 mm at the platform. Six passive joints quickly add up to 0.2–0.3 mm. Use **bearings or shoulder screws, not printed holes**.
* The servo spline (25 teeth, 14.4° per tooth) gives a mounting angle error of up to ±7°, which is why the per-servo trim exists.
* The gear backlash of the MG996R is about 0.5–1°, as large as its dead band.

## Electrical

```
              6 V / 5 A PSU ──┬──[1000 µF]──┬── servo 1 V+ (red)
                              │             ├── servo 2 V+
          e-stop (NC) ────────┘             └── servo 3 V+
                                                 servo GND ──┐
 USB (PC) ── Arduino 5V/GND ─── MPU6050 VCC/GND              │
             Arduino GND ────────────────────────────────────┴── COMMON GROUND (star point)
             D9/D10/D11 ── servo signal 1/2/3 (orange)
             A4 (SDA) / A5 (SCL) ── MPU6050 #1 (AD0=GND, 0x68, hand)
                                 └─ MPU6050 #2 (AD0=VCC, 0x69, base/platform, optional)
```

Rules (most "my servos jitter/reset" problems come from here):

1. **Never power the servos from the Arduino's 5 V pin.** Three MG996R can draw > 7 A at stall.
2. **Common ground** between the servo supply and the Arduino, at one star point.
3. 1000 µF low-ESR capacitor at the servo supply terminals.
4. The e-stop breaks the **servo supply** (hardware), not just a software signal (chapter 07).
5. Hand-held IMU cable: twisted pairs, ≤ 1 m, 2.2 kΩ pull-ups, I²C at 100 kHz. For longer cables use an I²C extender (P82B715) or put an MCU in the handle and use serial or CAN.

## Checklist

- [ ] All joints turn freely with no visible play
- [ ] Servo supply measured under load (> 5.5 V at 3 × 2 A)
- [ ] Common ground and capacitor in place
- [ ] E-stop cuts the servo power
