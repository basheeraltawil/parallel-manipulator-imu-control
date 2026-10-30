# From prototype to product: industrialization guide

This guide is for anyone who wants to build this robot for real: as a teaching kit, a lab instrument, or the start of a commercial XY-θ stage. It follows the order an engineering team would work in. Each chapter ends with a checklist.

| Phase | Chapter | Output |
|---|---|---|
| Define | [01 Requirements and specification](01_requirements_and_specs.md) | product spec with measurable numbers |
| Select | [02 Bill of materials and component selection](02_bill_of_materials.md) | BOM in three tiers (teaching / lab / industrial) |
| Build | [03 Mechanical and electrical build](03_mechanical_and_electrical.md) | printed or machined parts, wiring, power |
| Program | [04 Firmware and communication protocol](04_firmware_and_protocol.md) | flashed controller, protocol, ROS 2 link |
| Commission | [05 Calibration and commissioning](05_calibration_and_commissioning.md) | calibrated robot, commissioning record |
| Verify | [06 Testing and validation](06_testing_and_validation.md) | measured accuracy and repeatability (ISO 9283 style) |
| Protect | [07 Safety and compliance](07_safety_and_compliance.md) | risk assessment, safety functions, CE path |
| Scale | [08 Scaling to a product](08_scaling_to_product.md) | DFM, cost, roadmap |

**Rule of the guide:** simulate first, then build. Every requirement in chapter 01 can be checked in `simulation/` before you buy a part.
