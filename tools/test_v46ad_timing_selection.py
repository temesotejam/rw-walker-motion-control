#!/usr/bin/env python3
"""Historical entry point: selectable timing was removed in V46aj."""
from pathlib import Path
import runpy
runpy.run_path(str(Path(__file__).with_name('test_v46aj_fixed_timing.py')), run_name='__main__')
