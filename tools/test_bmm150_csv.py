"""Offline compensation agrees with independent Bosch SensorAPI float vectors."""
from pathlib import Path
import convert_rwlog_to_csv as converter

trim=dict(x1=-5,y1=4,x2=-3,y2=-5,xy1=40,xy2=-2,z1=32757,
          z2=-15000,z3=-500,z4=1000,xyz1=25000)
count=0
for line in (Path(__file__).parent/'fixtures/bmm150_bosch_float_vectors.inc').read_text().splitlines():
    v=[float(x.rstrip('f')) for x in line.strip('{},').split(',')]
    x,y,z,h=map(int,v[:4])
    aux=(x*8%65536).to_bytes(2,'little')+(y*8%65536).to_bytes(2,'little')+\
        (z*2%65536).to_bytes(2,'little')+(h*4|1).to_bytes(2,'little')
    values=converter.magnetic_values(aux,trim,1)
    assert values[5]==1
    assert all(abs(float(a)-b)<=1e-5*max(1,abs(b)) for a,b in zip(values[6:9],v[4:]))
    count+=1
assert count==64
assert converter.magnetic_values(bytes(8),trim,1)[5]==0
assert converter.magnetic_values(aux,None,1)[5]==0
assert converter.magnetic_values(aux,trim,0)[5]==0
print('BMM150 offline CSV compensation vs Bosch float, missing trim/sample/RHALL PASS')
