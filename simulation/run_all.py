#!/usr/bin/env python3
"""Regenerate every scenario result, GIF and video (about 10 minutes on a laptop).

    python3 simulation/run_all.py            # everything
    python3 simulation/run_all.py --no-media # numbers only (fast, used by CI)
"""

import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SCENARIOS = ["s0_overview", "s1_imu_teleoperation", "s2_precision_pick_place", "s3_contour_dispensing",
             "s4_base_motion_compensation", "s5_singularity_guard"]

if __name__ == "__main__":
    extra = sys.argv[1:]
    failed = []
    for s in SCENARIOS:
        if s == "s0_overview" and "--no-media" in extra:
            continue
        print(f"\n===== {s} =====", flush=True)
        if subprocess.run([sys.executable, str(HERE / "scenarios" / f"{s}.py"), *extra]).returncode:
            failed.append(s)
    sys.exit(f"failed: {failed}" if failed else 0)
