# 07 · Safety and compliance

> This robot is small and weak (about 1 N m per servo), but **pinch points and unexpected motion are real hazards**, especially in classrooms. Treat it as a machine.

## Risk assessment (ISO 12100 method, simplified)

| Hazard | Situation | Risk reduction implemented / required |
|---|---|---|
| Pinching between links | fingers in the workspace during motion | low speed limit (50 mm/s); **e-stop**; teaching rule "hands off while powered"; optional transparent cover |
| Unexpected motion at power-up | servos jump to their command on boot | firmware commands **home** before attaching; slew limit; power the servos *after* the logic |
| Unexpected motion after a comms loss | PC crash in stream mode | **watchdog** (0.5 s → hold) |
| Loss of control at a singularity | a commanded path crosses a Type-II singularity (S5) | **singularity guard** in the controller *and* the firmware |
| Out-of-range command | bug or typo sends an unreachable pose | firmware validation: reachability, ±85° travel |
| Electrical | 6 V high current, LiPo batteries | fused supply, no exposed terminals, LiPo charging rules |
| Thrown parts | broken printed link at high speed | speed limit; PETG; inspection after crashes |

## Safety functions (what an industrial version needs)

| Function | Standard reference | Tier 1 (kit) | Tier 3 (industrial) |
|---|---|---|---|
| Emergency stop | ISO 13850, stop category 0/1 | NC button breaking the servo supply | certified e-stop relay, STO on the drives |
| Safe torque off | IEC 61800-5-2 | — | STO in the servo drives |
| Speed and force limitation | ISO 10218-1, ISO/TS 15066 (collaborative) | software limit | safety-rated monitored speed |
| Protective stop / guarding | ISO 10218-2, ISO 13857 | — | interlocked guard or light curtain |
| Control-system reliability | ISO 13849-1 (PL), IEC 62061 (SIL) | — | PL d for the e-stop and guards |

## Compliance path (EU example)

* **Machinery Regulation (EU) 2023/1230** (applies from January 2027): risk assessment, technical file, declaration of conformity, CE marking.
* **EMC Directive 2014/30/EU** and the **Low Voltage Directive** where applicable (the PSU should be CE-marked).
* Educational kits can fall under the **Toy Safety Directive** if they target children under 14. This design is meant for adults and university use, so state that clearly.
* Elsewhere: comparable requirements (UKCA, FCC part 15 for the electronics, OSHA guidance for robots in the US).

## Checklist
- [ ] Risk assessment written and reviewed
- [ ] E-stop breaks the servo power and was tested
- [ ] Watchdog and singularity guard tested (chapter 05)
- [ ] User instructions contain safety warnings and intended use
