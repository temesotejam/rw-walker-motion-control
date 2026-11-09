"""Preprocess both opt-in modes and verify the worker and labels are exclusive."""
from pathlib import Path
import subprocess
root=Path(__file__).resolve().parents[1]
def preprocess(name,angle):
    src=(root/name).read_text(encoding='utf8')
    # This test checks conditional wiring, independent of device headers.
    src='\n'.join('' if line.lstrip().startswith('#include') else line for line in src.splitlines())
    return subprocess.run(['g++','-E','-P','-x','c++','-DFOOT_EARLY_TRIAL=1',
        '-DANGLE_EARLY_TRIAL='+str(angle),'-'],input=src,text=True,capture_output=True,check=True).stdout
angle=preprocess('src/main.cpp',1)
geometry=preprocess('src/main.cpp',0)
assert 'early_worker.begin()' not in angle and 'earlySubmitFrame(' not in angle
assert 'early_worker.begin()' in geometry and 'earlySubmitFrame(' in geometry
assert 'runner.update()' in angle and 'runner.update()' in geometry
angle_log=preprocess('src/psram_logger.cpp',1)
geometry_log=preprocess('src/psram_logger.cpp',0)
assert 'angle_early_trial' in angle_log and 'foot_early_trial' not in angle_log
assert 'angle_rate_early_trial' in angle_log and 'foot_geometry_early_trial' not in angle_log
assert 'foot_early_trial' in geometry_log and 'angle_early_trial' not in geometry_log
assert 'geometry_worker_enabled' in angle_log and 'pulse_width_changes_alongside_timing' in angle_log
print('angle/geometry mode conditional wiring PASS')
