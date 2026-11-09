#!/usr/bin/env python3
"""Compatibility entry point for the current production rate-baseline tests."""
import runpy
from pathlib import Path
runpy.run_path(str(Path(__file__).with_name('test_v46ai_rate_only.py')),run_name='__main__')
