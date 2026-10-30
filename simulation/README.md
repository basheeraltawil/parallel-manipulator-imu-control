# Desktop simulation (no ROS needed)

Physics simulation of the robot in **MuJoCo**, built from the *same* URDF that ROS uses (xacro → URDF → MjSpec + 2 loop-closure constraints + servo actuators).

```bash
pip install -r requirements.txt
python3 simulation/run_all.py                       # every scenario, GIF and MP4 (docs/media)
python3 simulation/scenarios/s2_precision_pick_place.py --no-media
```

| Module | Purpose |
|---|---|
| `pm3rrr_sim/mujoco_model.py` | `MujocoPm3rrr`: load the URDF, close the loops, servos, off-screen rendering |
| `pm3rrr_sim/runner.py` | 50 Hz controller loop: target → IK → quantisation → hobby-servo model → physics |
| `pm3rrr_sim/plot2d.py`, `media.py` | top-view drawings, GIF/MP4 export |
| `scenarios/s0…s5` | overview + the five scenarios ([docs/scenarios](../docs/scenarios/README.md)) |

Headless rendering uses OSMesa (`MUJOCO_GL=osmesa`, set automatically). On Ubuntu: `sudo apt install libosmesa6`.
