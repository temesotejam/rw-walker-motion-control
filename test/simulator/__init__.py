"""Use the research simulator modules during unittest discovery."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "analysis/simulator"))
