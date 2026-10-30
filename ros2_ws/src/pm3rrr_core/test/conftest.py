"""Make the package importable when pytest runs from the repository root (no install needed)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
