import json,subprocess,sys
from pathlib import Path
records=json.loads(Path('tools/fixtures/wheel_speed_31767392.json').read_text())['records']
rows=[r for r in records if r['valid'] and r['rpm'] is not None and r['negative_peak_deg'] is not None]
data=''.join(f"{r['side']} {r['rpm']} 1 {r['time_ms']} {r['positive_peak_deg']} {r['negative_peak_deg']}\n" for r in rows)
subprocess.run([sys.argv[1], 'replay'],input=data,text=True,check=True)
