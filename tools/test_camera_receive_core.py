"""Execute the production task-creation wrapper: isolate camera affinity only."""
from pathlib import Path
import subprocess
import tempfile
from test_foot_fast_metadata import function
ROOT = Path(__file__).resolve().parents[1]
PREAMBLE = r'''
#include <cassert>
#include <cstring>
#include "camera_task_priority_patch.h"
using BaseType_t=int;using UBaseType_t=unsigned;
using TaskFunction_t=void(*)(void*);using TaskHandle_t=void*;
using portMUX_TYPE=int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) (++*(x))
#define portEXIT_CRITICAL(x) (--*(x))
#define pdPASS 1
namespace Config {constexpr unsigned ROLLER_IO_TASK_PRIORITY=4;}
namespace RuntimeDiag {static unsigned calls;static void cameraDriverTask(unsigned,unsigned,bool){++calls;}}
static unsigned received_priority,received_stack;static int received_core,result=1;
static char task;
extern "C" BaseType_t __real_xTaskCreatePinnedToCore(TaskFunction_t,const char*,uint32_t stack,void*,
    UBaseType_t priority,TaskHandle_t* handle,BaseType_t core){
    received_priority=priority;received_stack=stack;received_core=core;
    if(handle)*handle=result==pdPASS?&task:nullptr;return result;
}
'''
CASES = r'''
int main(){
    TaskHandle_t handle=nullptr;
    assert(__wrap_xTaskCreatePinnedToCore(nullptr,"cam_task",2048,nullptr,2,&handle,0)==pdPASS);
    assert(received_priority==3 && received_stack==8192 && handle==&task);
    assert(g_patch_snapshot.created && g_camera_task==handle && !g_patch_mux);
    assert(received_core==EXPECTED_CORE && g_patch_snapshot.core==EXPECTED_CORE);
    assert(__wrap_xTaskCreatePinnedToCore(nullptr,"imu",4096,nullptr,6,&handle,1)==pdPASS);
    assert(received_priority==6 && received_stack==4096 && received_core==1);
    assert(__wrap_xTaskCreatePinnedToCore(nullptr,"control",6144,nullptr,4,&handle,1)==pdPASS);
    assert(received_priority==4 && received_stack==6144 && received_core==1);
    assert(RuntimeDiag::calls==1);
    result=0;assert(!__wrap_xTaskCreatePinnedToCore(nullptr,"cam_task",16384,nullptr,2,&handle,0));
    assert(!g_patch_snapshot.created && !g_camera_task && !handle && received_stack==16384);
    assert(RuntimeDiag::calls==2 && !g_patch_mux);
}
'''
def main():
    source=(ROOT/'src/camera_task_priority_patch.cpp').read_text(encoding='utf8')
    globals_=source[source.index('namespace {'):source.index('extern "C"')]
    code=PREAMBLE+globals_+function(source,'extern "C" BaseType_t __wrap_xTaskCreatePinnedToCore(')+CASES
    with tempfile.TemporaryDirectory(prefix='camera_receive_core_') as folder:
        p=Path(folder);(p/'test.cpp').write_text(code,encoding='utf8')
        for core in (0,1):
            subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-Wno-misleading-indentation',
                '-I'+str(ROOT/'src'),'-I'+str(ROOT/'tools/host_v46o'),'-DEXPECTED_CORE='+str(core),
                '-DFOOT_RECEIVE_CORE1_V31='+str(core),str(p/'test.cpp'),'-o',str(p/'test.exe')],check=True)
            subprocess.run([str(p/'test.exe')],check=True)
    print('PASS actual camera task wrapper: core0/core1, below-control priority, stack, IMU/control unchanged, creation failure')
if __name__=='__main__':main()
