#!/usr/bin/env python3
"""Check actual linked placement and a bounded hot-code footprint, not speed."""
import argparse
import json
from pathlib import Path
import re
import subprocess

HOT_METHODS = ('predict', 'updateAccel', 'pitchDegFromQuaternion', 'clampf',
               'norm', 'normalized', 'dot', 'quatMultiply', 'quatNormalized',
               'deltaQuat', 'predictedSpecificForceUpBody', 'skew', 'inverse3x3',
               'smoothConfidence', 'symmetrizeCovariance', 'injectErrorState',
               'applyResetJacobian')
REQUIRED = ('mekf6::Mekf6::predict(', 'mekf6::Mekf6::updateAccel(',
            'mekf6::Mekf6::pitchDegFromQuaternion(', 'encodeLogSample(',
            'log_quantization::scaledI16(')
UPDATE_REQUIRED = tuple('ExperimentRunner::' + method + '(' for method in (
    'updateFilterSeries', 'finishDeferredComparison', 'updateComparisonDisplayAngles',
    'updateDisplayedAngles', 'updateCurrentRollState')) + (
    'captureMekfAttitude(',
    'Adafruit_Madgwick::updateIMU(float, float, float, float, float, float)',
    'Adafruit_Madgwick::updateIMU(float, float, float, float, float, float, float)')
UPDATE_PREFIXES = UPDATE_REQUIRED + ('Adafruit_Madgwick::invSqrt(',
    '(anonymous namespace)::updateDerivedAccel(', 'float madgwick_pitch::readPitchDeg<')


def audit(symbol_table):
    symbols = {}
    update_symbols = {}
    for line in symbol_table.splitlines():
        match = re.match(r'^([0-9a-fA-F]+)\s+.*?\bF\s+(\S+)\s+([0-9a-fA-F]+)\s+(.+)$', line)
        if not match:
            continue
        address, section, size, name = match.groups()
        hot = (any(name.startswith('mekf6::Mekf6::' + method + '(') for method in HOT_METHODS)
               or name.startswith(('encodeLogSample(', '(anonymous namespace)::quantize(',
                                   'log_quantization::scaledI16('))
               or ('mekf6::(anonymous namespace)::dot' in name))
        update = name.startswith(UPDATE_PREFIXES)
        if not hot and not update:
            continue
        if not section.startswith('.iram'):
            raise ValueError('Hot routine is outside IRAM: ' + name + ' in ' + section)
        if int(size, 16) <= 0:
            raise ValueError('Empty hot routine: ' + name)
        (update_symbols if update else symbols)[name] = {
            'address': '0x' + address, 'section': section, 'bytes': int(size, 16)}
    for prefix in REQUIRED:
        if not any(name.startswith(prefix) for name in symbols):
            raise ValueError('Missing required IRAM routine: ' + prefix)
    total = sum(s['bytes'] for s in symbols.values())
    if total > 16384:
        raise ValueError('Hot routine code exceeds 16 KiB: ' + str(total))
    for prefix in UPDATE_REQUIRED:
        if not any(name.startswith(prefix) for name in update_symbols):
            raise ValueError('Missing required normal-update IRAM routine: ' + prefix)
    update_total = sum(s['bytes'] for s in update_symbols.values())
    if update_total > 12288:
        raise ValueError('Normal-update function code exceeds 12 KiB: ' + str(update_total))
    return {'revision': 'targeted_iram_04720', 'symbols': symbols,
            'function_code_bytes': total, 'function_code_limit_bytes': 16384,
            'normal_update_symbols': update_symbols,
            'normal_update_code_bytes': update_total, 'normal_update_code_limit_bytes': 12288,
            'scope': 'function bodies;not_literal_pool_or_total_IRAM_size',
            'external_calls_and_data_may_use_flash': True,
            'hardware_timing_verified': False}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('elf', type=Path)
    parser.add_argument('--objdump', required=True)
    args = parser.parse_args()
    table = subprocess.check_output([args.objdump, '-t', '-C', str(args.elf)], text=True)
    print(json.dumps(audit(table), indent=2))
