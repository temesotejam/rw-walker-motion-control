"""Execute the production log-full early return against small host doubles.

The tested branch is extracted from ExperimentRunner::logSampleNow. The stop
and later record operations are doubles; this is a terminal-reason regression,
not a physical motor-stop or full LogSample encoder test.
"""
import hashlib
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def full_branch(source):
    start = source.index('void ExperimentRunner::logSampleNow()')
    record = source.index('LogSample row{};', start)
    prefix = source[start:record]
    matches = list(re.finditer(r'if\s*\(\s*logger_->full\(\)\s*\)\s*\{', prefix))
    assert len(matches) == 1, 'Expected one production full-buffer early return'
    begin = matches[0].start()
    # Exclude braces inside comments or string literals from block balancing.
    tokens = re.finditer(r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]',
                         prefix[matches[0].end() - 1:])
    depth = 0
    for token in tokens:
        if token.group() == '{': depth += 1
        elif token.group() == '}':
            depth -= 1
            if depth == 0:
                end = matches[0].end() - 1 + token.end()
                block = prefix[begin:end]
                assert re.search(r'\breturn\s*;', block), 'Full log must not reach record processing'
                return block
    raise AssertionError('Unclosed production log-full branch')


FIXTURE = r'''
#include <cassert>
#include <cstdint>
#include <iostream>
#include <string>
struct Logger {
  unsigned used=0,capacity=2;
  bool full()const{return used>=capacity;}
};
struct Completion {
  bool pending=true,finished=false;
  unsigned count=0;uint32_t elapsed_ms=0;uint8_t reason=0;
};
class Runner {
 public:
  struct Status {bool emergency_stop=false,running=true,pulse_active=true;
    int motor_command_ma=1200;std::string last_error;};
  Logger log;Logger* logger_=&log;Status status_;Completion completion;
  unsigned stop_calls=0,records=0;
  void requestEmergencyStop(const char* reason){
    ++stop_calls;status_.motor_command_ma=0;status_.pulse_active=false;
    if(completion.pending){completion.pending=false;completion.finished=true;
      ++completion.count;completion.elapsed_ms=17;completion.reason=3;}
    status_.emergency_stop=true;status_.running=false;status_.last_error=reason;
  }
  void logSampleNow(){
__PRODUCTION_FULL_BRANCH__
    // Double for the record/encode/append work after production's full branch.
    ++records;++log.used;
  }
};
void checkCompletion(const Completion& a,const Completion& b){
  assert(a.pending==b.pending && a.finished==b.finished && a.count==b.count &&
    a.elapsed_ms==b.elapsed_ms && a.reason==b.reason);
}
int main(){
  // A running full logger still stops and never reaches row processing.
  {Runner r;r.log.used=r.log.capacity;r.logSampleNow();
    assert(r.stop_calls==1 && r.records==0 && r.status_.emergency_stop && !r.status_.running);
    assert(r.status_.last_error=="log_buffer_full" && !r.status_.pulse_active && r.status_.motor_command_ma==0);
    assert(r.completion.finished && !r.completion.pending && r.completion.count==1 && r.completion.reason==3);
    const auto completed=r.completion;r.logSampleNow();
    assert(r.stop_calls==1 && r.records==0);checkCompletion(r.completion,completed);}
  // A motion stop has already stopped/completed the pulse. A final full-log
  // attempt must preserve its first cause, stop count and completion snapshot.
  {Runner r;r.requestEmergencyStop("fore_aft_outward_divergence");
    const auto completed=r.completion;const auto reason=r.status_.last_error;
    const auto calls=r.stop_calls;r.log.used=r.log.capacity;
    for(unsigned i=0;i<3;++i)r.logSampleNow();
    assert(r.status_.last_error==reason && r.stop_calls==calls && r.records==0);
    assert(r.status_.emergency_stop && !r.status_.running && !r.status_.pulse_active && r.status_.motor_command_ma==0);
    checkCompletion(r.completion,completed);}
  // Empty and partially occupied logs pass this branch into record work.
  {Runner r;const auto before=r.completion;r.logSampleNow();
    assert(r.log.used==1 && r.records==1 && r.stop_calls==0 && !r.status_.emergency_stop);
    assert(r.status_.running && r.status_.last_error.empty());checkCompletion(r.completion,before);
    r.logSampleNow();assert(r.log.used==2 && r.records==2 && r.stop_calls==0);
    r.logSampleNow();assert(r.records==2 && r.stop_calls==1 && r.status_.last_error=="log_buffer_full");}
  // A non-full log also permits the existing ESTOP's final diagnostic record.
  {Runner r;r.requestEmergencyStop("fore_aft_outward_divergence");const auto completed=r.completion;
    r.logSampleNow();assert(r.records==1 && r.stop_calls==1 && r.status_.last_error=="fore_aft_outward_divergence");
    checkCompletion(r.completion,completed);}
  std::cout<<"terminal stop reason: production log-full branch PASS\n";
}
'''


def main():
    source = (ROOT / 'src/experiment_runner.cpp').read_text(encoding='utf8')
    block = full_branch(source)
    with tempfile.TemporaryDirectory(prefix='terminal_stop_reason_') as folder:
        temp = Path(folder)
        (temp / 'test.cpp').write_text(FIXTURE.replace('__PRODUCTION_FULL_BRANCH__', block), encoding='utf8')
        exe = temp / 'test.exe'
        subprocess.run(['g++', '-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror',
                        str(temp / 'test.cpp'), '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
    print('production full-branch SHA256', hashlib.sha256(block.encode()).hexdigest())


if __name__ == '__main__':
    main()
