import json,subprocess,sys,math
from pathlib import Path
d=json.loads(Path('tools/fixtures/walking_loss_50881356.json').read_text())
r=d['records'];assert len(r)==32
assert all(x['pulse_ms']==0 and not x['output'] for x in r)
assert all(5000<=x['zero_ms']<x['peak_ms']<30000 for x in r)
text=''.join(' '.join(str(x[k]) for k in ('rate_dps','rpm','vbat_mv','previous_deg','peak_deg'))+'\n' for x in r)
subprocess.run([sys.argv[1],'replay'],input=text,text=True,check=True)

# These are observation-to-prediction fit checks, not independent validation.
def potential(a):
    a=abs(a)*math.pi/180;c=math.sqrt(.15**2-.005**2)
    dh=(-2*.12*math.sin(a/2)**2+.005*math.sin(a) if a<=math.asin(.005/.15)
        else .15-c+2*(c-.12)*math.sin(a/2)**2)
    return .1997*9.80665*dh*1000
x=[v['rate_dps']**2 for v in r];y=[potential(v['peak_deg']) for v in r]
xm=sum(x)/len(x);ym=sum(y)/len(y)
b=sum((a-xm)*(v-ym) for a,v in zip(x,y))/sum((a-xm)**2 for a in x);a=ym-b*xm
assert abs(a-.09046158)<1e-7 and abs(b-.00018973)<1e-8
loss=sum(potential(v['previous_deg'])-potential(v['peak_deg']) for v in r)/len(r)
assert abs(loss-.1401290743)<1e-6
print('32 measured unpowered half-cycles, fitted coefficients and mean effective decrease PASS')

second=json.loads(Path('tools/fixtures/walking_loss_39418856.json').read_text())['records']
assert len(second)==31 and all(v['pulse_ms']==0 and not v['output'] for v in second)
errors=[.09046158+.00018973*v['rate_dps']**2-potential(v['peak_deg']) for v in second]
assert abs(sum(errors)/len(errors))<.002
assert (sum(e*e for e in errors)/len(errors))**.5 < .021
assert all(24.7827<=v['rate_dps']<=36.752 and 511.45<=v['rpm']<=603.21 and 7000<=v['vbat_mv']<=8120 for v in second)
print('Second run: 31 unpowered arrivals, held-out coast accuracy and expanded bounds PASS')

third=json.loads(Path('tools/fixtures/walking_loss_47647215.json').read_text())['records']
assert len(third)==31 and all(v['pulse_ms']==0 and not v['output'] for v in third)
errors=[.09046158+.00018973*v['rate_dps']**2-potential(v['peak_deg']) for v in third]
assert abs(sum(errors)/len(errors)+.02013527)<.00001
assert (sum(e*e for e in errors)/len(errors))**.5 < .031
assert all(24.7827<=v['rate_dps']<=36.875 and 446.5<=v['rpm']<=603.21 and 4.3006<=v['previous_deg']<=6.11884 for v in third)
text=''.join(' '.join(str(x[k]) for k in ('rate_dps','rpm','vbat_mv','previous_deg','peak_deg'))+'\n' for x in third)
subprocess.run([sys.argv[1],'third'],input=text,text=True,check=True)
print('Third run: 31 independent unpowered arrivals and expanded bounds PASS')

fourth=json.loads(Path('tools/fixtures/walking_42306015.json').read_text())['records']
unpowered=[v for v in fourth if v['pulse_ms']==0 and not v['output']]
powered=[v for v in fourth if v['pulse_ms']==25 and v['output']]
assert len(unpowered)==18 and len(powered)==13
errors=[.09046158+.00018973*v['rate_dps']**2-potential(v['peak_deg']) for v in unpowered]
assert (sum(e*e for e in errors)/len(errors))**.5 < .043
assert all(18.45<=v['rate_dps']<=37.30 and 362<=v['rpm']<=603.21 and 4.3006<=v['previous_deg']<=6.90 for v in unpowered+powered)
text=''.join(' '.join(str(x[k]) for k in ('rate_dps','rpm','vbat_mv','previous_deg','peak_deg'))+'\n' for x in unpowered)
subprocess.run([sys.argv[1],'fourth'],input=text,text=True,check=True)
print('Fourth run: 18 unpowered replay, 13 powered observations and expanded bounds PASS')

# Latest run: low-rpm unsupported coasts; no causal estimate of powered gain.
fifth=json.loads(Path('tools/fixtures/walking_55836070.json').read_text())['records']
coast=[v for v in fifth if not v['output'] and v['pulse_ms']==0 and 323.9<=v['rpm']<362 and 18.45<=v['rate_dps']<=37.3]
assert len(coast)==8
errors=[.09046158+.00018973*v['rate_dps']**2-potential(v['peak_deg']) for v in coast]
assert (sum(e*e for e in errors)/len(errors))**.5 < .025
assert len([v for v in fifth if v['pulse_ms']==40 and v['output']])==15
print('Fifth run: eight low-rpm unpowered observations and 15 capped powered arrivals PASS')

# Additional low-rate, low-rpm walking coasts; not powered validation.
new=json.loads(Path('tools/fixtures/walking_317_275_338.json').read_text())['records']
assert len(new)==13 and all(not v['output'] and v['pulse_ms']==0 for v in new)
assert all(13.75<=v['rate_dps']<=37.8 and 274.9<=v['rpm']<=603.21 and 4.3006<=v['previous_deg']<=7.65 for v in new)
errors=[.09046158+.00018973*v['rate_dps']**2-potential(v['peak_deg']) for v in new]
assert (sum(e*e for e in errors)/len(errors))**.5 < .031
print('Eight runs: thirteen low-speed unpowered observations PASS')

# Observed 60 ms arrivals inform the additional speed gate; these samples do
# not validate motor input durations beyond 60 ms or establish causation.
latest=json.loads(Path('tools/fixtures/walking_60ms_arrivals_502_276.json').read_text())['records']
assert len(latest)>=25
fresh=[r for r in latest if r['peak_rpm_valid'] and r['peak_speed_age_us']<=30000]
assert len(fresh)>=25 and sum(r['peak_rpm']>=30 and r['peak_deg']<r['target_deg'] for r in fresh)>=20
print('Recorded 60 ms arrivals: fresh peak speed and below-target evidence PASS')

# Four-run post-hoc coast matches; samples within a run are correlated and
# the set contains no powered observations in the new domain.
low=json.loads(Path('tools/fixtures/walking_low_rpm_139_319_443_387.json').read_text())['records']
assert len(low)==26 and len({r['run'] for r in low})==4
assert all(not r['output'] and r['pulse_ms']==0 and r['posthoc_match'] for r in low)
assert all(240<=r['rpm']<274.9 and 13.75<=r['rate_dps']<=37.8 and 4.3006<=r['previous_deg']<=7.65 for r in low)
errors=[.09046158+.00018973*r['rate_dps']**2-potential(r['peak_deg']) for r in low]
assert (sum(x*x for x in errors)/len(errors))**.5<.028
text=''.join(' '.join(str(r[k]) for k in ('rate_dps','rpm','vbat_mv','previous_deg','peak_deg'))+'\n' for r in low)
subprocess.run([sys.argv[1],'low'],input=text,text=True,check=True)
print('Four runs: 26 post-hoc low-rpm coast arrivals; powered effect not established PASS')
