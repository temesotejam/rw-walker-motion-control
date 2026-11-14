"""Compile real v17 JSON serialization, validate unique keys and CSV fields."""
from pathlib import Path
import csv
import subprocess
import tempfile
import convert_rwlog_to_csv as converter

ROOT = Path(__file__).resolve().parents[1]
base = ROOT / 'tools/test_energy_pulse_lifecycle_metadata_v16.py'
context = {'__file__': str(base)}
exec(compile(base.read_text(encoding='utf8').split('\nwith tempfile.TemporaryDirectory', 1)[0], str(base), 'exec'), context)
fixture = '#include "walking_return_energy.h"\n' + context['fixture']
fixture = fixture.replace('a.requested_ms=a.guarded_ms=80;',
    'a.arrival_bias_j=.000123f;a.return_work_j=.000234f;a.return_event=i==1;'
    'a.return_deceleration_dps2=120;a.return_stopping_distance_deg=.75f;a.return_foot_age_ms=75;'
    'a.requested_ms=a.guarded_ms=80;')
fixture = fixture.replace('for(unsigned branch:{0u,15u,19u,20u,21u})',
                          'for(unsigned branch:{0u,15u,19u,20u,21u,22u})')
with tempfile.TemporaryDirectory(prefix='return_metadata_v17_') as folder:
    temp = Path(folder)
    (temp / 'test.cpp').write_text(fixture, encoding='utf8')
    exe = temp / 'test.exe'
    subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-DEARLY_ENERGY_MAIN=1', '-DPULSE_LIFECYCLE_FIX=1', '-DWALKING_RETURN_ENERGY=1',
                    '-I' + str(ROOT / 'tools/host_v46o'), '-I' + str(ROOT / 'src'),
                    str(temp / 'test.cpp'), '-o', str(exe)], check=True)
    for mode, count in [('', 7), ('empty', 0)]:
        output = subprocess.check_output([str(exe)] + ([mode] if mode else []), text=True).splitlines()
        raw, _ = map(context['parse'], output)
        assert raw['energy_model'] == raw['wheel_work_model_revision'] == 'walking_return_energy_v17'
        assert raw['pulse_lifecycle']['energy_model_changed'] is True
        assert raw['work_control_branch_codes']['22'] == 'walking_return_energy_v17'
        assert raw['walking_energy']['rescue_ceiling_ms'] == 80
        assert raw['walking_energy']['branch'] == 22
        assert raw['walking_energy']['energy_neutral_model'].startswith('nominal_only;')
        assert 'rescue15' not in raw['energy_control_autonomous_policy']
        for e in raw['event_revision_checks']:
            if e['branch'] == 22:
                assert e['arrival_energy_model_revision'] == 'walking_return_energy_v17'
        metadata = converter.expand_tables(raw)
        out = temp / (mode or 'rows')
        out.mkdir()
        assert converter.write_walking_energy_diagnostics(metadata, out) == count
        if count:
            with (out / 'walking_energy.csv').open(newline='', encoding='utf8') as stream:
                rows = list(csv.DictReader(stream))
            assert abs(float(rows[0]['arrival_bias_j']) - .000123) < 1e-9
            assert abs(float(rows[0]['return_work_j']) - .000234) < 1e-9
            assert rows[1]['return_event'] == 'True' and rows[0]['return_event'] == 'False'
            assert float(rows[1]['return_foot_age_ms']) == 75
print('v17 real metadata: revisions/budgets/neutral limits/completion/compact table/CSV PASS')
