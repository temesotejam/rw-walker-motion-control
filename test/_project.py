"""Allow host tests to import the same tools used for recorded experiments."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
for directory in ("tools", "analysis/support", "analysis/measurement"):
    sys.path.append(str(ROOT / directory))
