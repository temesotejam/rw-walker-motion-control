import json,subprocess,sys
from pathlib import Path
d=json.loads(Path('tools/fixtures/low_peak_run175_replay.json').read_text())
assert len(d['events'])>80
r=subprocess.run([sys.argv[1]],input='\n'.join(d['events'])+'\n',text=True,capture_output=True,check=True)
assert 'first_changed_ms=12304 baseline=80 candidate=78' in r.stdout, r.stdout
print('Recorded run 175: first selected action changes from 80 to 78ms; no post-change trajectory is inferred PASS')
