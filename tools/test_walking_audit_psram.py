"""Run v16's real motor/completion path with a PSRAM archive and allocation loss."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = ROOT / 'tools/test_energy_main_lifecycle_v16.py'
namespace = {'__file__': str(source)}
prefix = source.read_text(encoding='utf8').split('\nwith tempfile.TemporaryDirectory(', 1)[0]
exec(compile(prefix, str(source), 'exec'), namespace)
fixture = namespace['fixture']
allocator = r'''
#include "psram_archive.h"
#include <cstdlib>
static bool allocation_allowed=true;
static unsigned allocations=0;
static void* allocate_archive(size_t size){++allocations;return allocation_allowed?std::malloc(size):nullptr;}
'''
fixture = allocator + fixture
marker = 'class ExperimentRunner {public:'
assert fixture.count(marker) == 1
fixture = fixture.replace(marker, marker + '\n ExperimentRunner(){walking_energy_audit_.begin(allocate_archive);}\n')
marker = 'int main(){'
assert fixture.count(marker) == 1
fixture = fixture.replace(marker, marker + r'''
 // The active completion state stays valid even if the passive archive fails.
 allocation_allowed=false;
 { ExperimentRunner r;assert(!r.walking_energy_audit_.available());
   ready(r);r.roller_obj.write_delay_ms=7;const auto before=allocations;
   r.acceptOptionalEarlyMotion(1200,55,-.5f);
   assert(r.status_.pulse_active && r.walking_energy_completion_pending_);
   assert(r.walking_energy_count_==0 && r.walking_energy_total_==1 && r.walking_energy_active_audit_==-1);
   const auto width=r.status_.pulse_width_ms_setting;
   mock_ms=r.active_pulse_start_ms_+width;r.updateEnergyControlAutonomousPulse(r.run_start_ms_+1200);
   const auto& a=r.walking_energy_active_pulse_;
   assert(!r.status_.pulse_active && !r.walking_energy_completion_pending_ && a.completion.valid);
   assert(a.completion.actual_elapsed_ms==width && a.completion_reason==1);
   assert(close(r.energy_control_autonomous_pending_q_command_mA_s_,a.completion.q_mas));
   assert(r.predicted_current_end_ms_==r.run_start_ms_+a.output_end_ms);
   assert(allocations==before); // No allocation from admission or completion.
 }
 allocation_allowed=true;
 { ExperimentRunner r;const auto before=allocations;
   assert(r.walking_energy_audit_.begin(allocate_archive) && allocations==before);
   assert(r.walking_energy_audit_.available());
   assert(sizeof(r.walking_energy_audit_)==sizeof(void*));
   std::cout<<"archive bytes="<<r.walking_energy_audit_.bytes()<<" internal handle="<<sizeof(r.walking_energy_audit_)<<"\n";
 }
''')

with tempfile.TemporaryDirectory(prefix='walking_audit_psram_') as directory:
    tmp = Path(directory)
    (tmp / 'test.cpp').write_text(fixture, encoding='utf8')
    executable = tmp / 'test.exe'
    subprocess.run(['g++', '-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-DFOOT_SUPPORT_ASSIST=1', '-DFOOT_SUPPORT_LOCAL=1', '-DFOOT_SUPPORT_TRANSFER=1',
                    '-DFOOT_SUPPORT_ENERGY=1', '-DEARLY_ENERGY_MAIN=1', '-DPULSE_LIFECYCLE_FIX=1',
                    '-DWALKING_AUDIT_PSRAM=1', '-DSWAY_BALANCE_TRIAL=1',
                    '-I'+str(ROOT/'src'), str(tmp/'test.cpp'), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
print('PASS: startup-only PSRAM archive; real control/completion with full, overflow and unavailable archive')

# Exercise the actual compact metadata serializer with a populated archive and
# with startup allocation unavailable. Keep existing record/CSV field meanings.
source = ROOT / 'tools/test_energy_pulse_lifecycle_metadata_v16.py'
namespace = {'__file__': str(source)}
prefix = source.read_text(encoding='utf8').split('\nwith tempfile.TemporaryDirectory(', 1)[0]
exec(compile(prefix, str(source), 'exec'), namespace)
metadata = allocator + namespace['fixture']
marker = 'WalkingEnergyAudit walking_energy_audit_[192]'
assert metadata.count(marker) == 1
metadata = metadata.replace(marker, 'PsramArchive<WalkingEnergyAudit,192> walking_energy_audit_')
marker = 'class ExperimentRunner {public:'
assert metadata.count(marker) == 1
metadata = metadata.replace(marker, marker + '\n ExperimentRunner(){walking_energy_audit_.begin(allocate_archive);}\n')
marker = 'ExperimentRunner runner;'
assert metadata.count(marker) == 1
metadata = metadata.replace(marker, 'allocation_allowed=argc<2;'+marker)
with tempfile.TemporaryDirectory(prefix='walking_archive_metadata_') as directory:
    tmp = Path(directory)
    (tmp / 'test.cpp').write_text(metadata, encoding='utf8')
    executable = tmp / 'test.exe'
    subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-DEARLY_ENERGY_MAIN=1', '-DPULSE_LIFECYCLE_FIX=1', '-DWALKING_AUDIT_PSRAM=1',
                    '-I'+str(ROOT/'tools/host_v46o'), '-I'+str(ROOT/'src'),
                    str(tmp/'test.cpp'), '-o', str(executable)], check=True)
    for arguments, available, count in [([], True, 7), (['empty'], False, 0)]:
        lines = subprocess.check_output([str(executable)]+arguments, text=True).splitlines()
        data = namespace['parse'](lines[0])['walking_energy']
        assert data['archive_storage'] == 'psram'
        assert data['archive_available'] is available
        assert data['archive_bytes'] == 39936
        assert data['stored'] == count
print('PASS: real PSRAM archive metadata: populated and explicit unavailable/empty')
