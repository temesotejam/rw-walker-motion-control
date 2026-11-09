#!/usr/bin/env python3
from verify_realtime_code_link import audit, REQUIRED, UPDATE_REQUIRED

rows = [f'40378000 g     F .iram0.text 00000100 {name}args)' for name in REQUIRED]
rows += [f'40380000 g F .iram0.text 00000100 {name}' for name in UPDATE_REQUIRED]
valid = '\n'.join(rows + ['42010000 g     F .flash.text 00000040 mekf6::Mekf6::reset()'])
assert audit(valid)['function_code_bytes'] == 1280
for bad in (valid.replace('.iram0.text', '.flash.text', 1),
            '\n'.join(rows[1:]),
            valid.replace('00000100', '00010000', 1),
            valid.replace('00000100 ExperimentRunner::', '00010000 ExperimentRunner::', 1),
            valid.replace('.iram0.text 00000100 Adafruit_Madgwick::', '.flash.text 00000100 Adafruit_Madgwick::', 1),
            valid + '\n42010100 l F .flash.text 00000040 mekf6::Mekf6::deltaQuat(args)'):
    try:
        audit(bad)
    except ValueError:
        pass
    else:
        raise AssertionError('Invalid linked placement / footprint accepted')
print('IRAM audit rejects missing/misplaced helpers and excess code; cold flash code allowed PASS')
