"""Make pm3rrr_core and pm3rrr_control importable from a source checkout (no install needed)."""
import sys
from pathlib import Path

src = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(src / "pm3rrr_core"))
sys.path.insert(0, str(src / "pm3rrr_control"))
