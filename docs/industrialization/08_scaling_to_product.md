# 08 · Scaling to a product

## Design for manufacturing

| Volume | Links and base | Joints | Notes |
|---|---|---|---|
| 1–10 | FDM printed PETG (this repo) | 624ZZ bearings, shoulder screws | hours per unit; tolerance ±0.1–0.2 mm |
| 10–500 | laser-cut 3 mm aluminium or MJF nylon | bearings pressed into reamed seats | fixtures for assembly; jigs for the servo trim |
| 500+ | injection-moulded glass-filled nylon or die-cast | bearings moulded in | tooling cost ≈ 10–30 k€; design draft angles and ribs |

Keep the **layered stack-up** (lesson 02). It makes self-collision impossible and simplifies assembly.

## Cost model (tier 2 lab version, 100 units, indicative)

| Item | € / unit |
|---|---|
| 3 smart servos + adapter | 150 |
| Mechanics (cut/printed parts, bearings, fasteners) | 40 |
| Electronics (MCU, IMU, PSU, e-stop, enclosure) | 45 |
| Assembly and calibration (1.5 h) | 60 |
| Packaging, documentation, warranty reserve | 25 |
| **Cost of goods** | **≈ 320 €** → sale price 2–3× |

## Product roadmap

1. **v1.0 (this repo):** teaching kit, open source, simulations and course.
2. **v1.1:** smart servos + `ros2_control` hardware interface + camera calibration tool.
3. **v2.0:** machined lab stage with encoders on the passive joints (assembly-mode monitoring), micro-ROS firmware, CE file.
4. **Research add-ons:** redundant 4th leg (removes the internal singularities), force sensing, visual servoing.

## Documentation a product needs

* User manual (installation, operation, safety, maintenance)
* Technical file (risk assessment, drawings, BOM, test reports, declaration of conformity)
* Service manual (calibration procedure = chapter 05, spare parts)
* Software release notes and versioned firmware (git tags; CI builds the release artefacts)

## Checklist
- [ ] Volume and target cost fixed
- [ ] Manufacturing process chosen per part
- [ ] Documentation set started
