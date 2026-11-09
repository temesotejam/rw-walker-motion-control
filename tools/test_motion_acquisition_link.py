#!/usr/bin/env python3
from verify_motion_acquisition_link import audit, REQUIRED
rows=[f'40380000 g F .iram0.text 00000100 {name}args)' for name in REQUIRED]
valid='\n'.join(rows)
assert audit(valid)['function_code_bytes']==len(REQUIRED)*256
for bad in (valid.replace('.iram0.text','.flash.text',1), '\n'.join(rows[1:]),
            valid.replace('00000100','00010000',1),
            valid+'\n42000000 l F .flash.text 00000020 direct_q::width(float)::{lambda()#1}::operator()() const'):
    try: audit(bad)
    except ValueError: pass
    else: raise AssertionError('Accepted invalid placement/size')
print('Motion/acquisition link audit: required functions, emitted lambdas and 25 KiB limit PASS')
