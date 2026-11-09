#!/usr/bin/env python3
"""Check linked motion, acquisition and lateral energy placement/size."""
import argparse
import json
from pathlib import Path
import re
import subprocess
import sys

MOTION_METHODS = '''resetEnergyControlAutonomousPeakTracker updateEnergyControlAutonomousMotion
updateEnergyControlAutonomousPeakTracker recordEnergyControlAutonomousPeak
updateEnergyControlAutonomousAtZeroCross updateEnergyControlAutonomousPulse
beginEnergyControlAutonomousPulse beginEnergyControlAutonomousStartKickPulse
energyControlAutonomousGainForSide energyControlAutonomousCorrectionParameters
predictedChargeMaS predictCurrentGoalMa predictRiseTauS predictBetaMin
updatePulseModelPrediction startTimingProbe maybeFinalizeTimingProbe stopActivePulse
stopMotor update logSampleIfDue logSampleNow'''.split()
ACQUISITION_METHODS = 'acquisitionLoop captureSensor recordPollProfile publishSample update'.split()
# These three small helpers can disappear when inlined into their hot callers.
# Check their placement if emitted, without requiring extra function calls.
INLINE_HELPERS = ('energyControlAutonomousGainForSide',
                  'energyControlAutonomousCorrectionParameters', 'predictBetaMin')
REQUIRED = tuple('ExperimentRunner::'+n+'(' for n in MOTION_METHODS if n not in INLINE_HELPERS) + tuple(
    'ImuManager::'+n+'(' for n in ACQUISITION_METHODS) + (
    'imu_i2c::read(', 'lateral_energy::potential(')
PREFIXES = REQUIRED + tuple('ExperimentRunner::'+n+'(' for n in INLINE_HELPERS) + (
    'direct_q::target(', 'direct_q::width(', 'ImuPollProfile::record(',
    'ExperimentRunner::tryEarlyFootCandidate(', 'ExperimentRunner::tryAngleEarlyCandidate(')
# 1.0.23 adds a bounded one-peak work dispatch at accepted ZERO crossings.
# Its fitted candidate scan is a separate flash routine; the audited IRAM
# motion/acquisition bodies use 24,717 bytes.  Reserve 25 KiB for these bodies
# while continuing to enforce placement and a finite headroom (~0.9 KiB).
LIMIT = 25600
COLD_BY_HOT = {
    "ExperimentRunner::tryEarlyFootCandidate(": "ExperimentRunner::decideEarlyFootCandidate(",
    "ExperimentRunner::tryAngleEarlyCandidate(": "ExperimentRunner::decideAngleEarlyCandidate(",
}


def audit(table):
    symbols = {}
    cold_symbols = {}
    for line in table.splitlines():
        m = re.match(r'^([0-9a-fA-F]+)\s+.*?\bF\s+(\S+)\s+([0-9a-fA-F]+)\s+(.+)$', line)
        if not m:
            continue
        address, section, size, name = m.groups()
        if name.startswith(tuple(COLD_BY_HOT.values())):
            if not section.startswith('.flash') or int(size, 16) <= 0:
                raise ValueError('Early cold routine must be in flash: '+name+' in '+section)
            cold_symbols[name] = dict(address='0x'+address, section=section, bytes=int(size,16))
            continue
        if not name.startswith(PREFIXES):
            continue
        if not section.startswith('.iram') or int(size, 16) <= 0:
            raise ValueError('Missing internal executable code: '+name+' in '+section)
        symbols[name] = dict(address='0x'+address, section=section, bytes=int(size,16))
    for prefix in REQUIRED:
        if not any(n.startswith(prefix) and '::{lambda' not in n for n in symbols):
            raise ValueError('Missing required routine: '+prefix)
    for hot,cold in COLD_BY_HOT.items():
        if any(n.startswith(hot) for n in symbols) and not any(n.startswith(cold) for n in cold_symbols):
            raise ValueError('Missing early threshold-reached cold routine: '+cold)
    size = sum(s['bytes'] for s in symbols.values())
    if size > LIMIT:
        print(json.dumps({'measured_symbols': symbols, 'function_code_bytes': size,
                          'function_code_limit_bytes': LIMIT, 'cold_symbols': cold_symbols}, indent=2), file=sys.stderr)
        raise ValueError('Motion/acquisition code exceeds 25 KiB: '+str(size))
    return dict(revision='lateral_energy_motion_link_v2_work_control', symbols=symbols,
                cold_symbols=cold_symbols,
                function_code_bytes=size, function_code_limit_bytes=LIMIT,
                scope='function bodies, including emitted lambdas; excludes literals and total IRAM',
                external_calls_and_data_may_use_flash=True, hardware_timing_verified=False)


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('elf', type=Path); p.add_argument('--objdump', required=True)
    a=p.parse_args()
    print(json.dumps(audit(subprocess.check_output([a.objdump,'-t','-C',str(a.elf)],text=True)),indent=2))
