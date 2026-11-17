"""Execute actual stopped queue cleanup with empty/full queues; no DMA access."""
from pathlib import Path
import subprocess
import tempfile
from test_foot_fast_metadata import function

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT/'third_party/esp32_camera_f0bb429/cam_hal.c').read_text(encoding='utf8')
code = r'''
#include <cassert>
#include <deque>
#include <vector>
struct camera_fb_t { int id; };
std::deque<camera_fb_t*> queue;
std::vector<camera_fb_t*> returned;
struct Camera { void* frame_buffer_queue; } camera{&queue};
Camera* cam_obj=&camera;
constexpr int pdTRUE=1;
int xQueueReceive(void* handle, camera_fb_t** out, unsigned timeout){
 assert(handle==&queue && timeout==0);
 if(queue.empty())return 0;
 *out=queue.front();queue.pop_front();return pdTRUE;
}
void cam_give(camera_fb_t* frame){returned.push_back(frame);}
''' + function(source, 'void rw_camera_discard_queued(') + r'''
int main(){
 cam_obj=nullptr;rw_camera_discard_queued();
 cam_obj=&camera;camera.frame_buffer_queue=nullptr;rw_camera_discard_queued();
 camera.frame_buffer_queue=&queue;rw_camera_discard_queued();assert(returned.empty());
 camera_fb_t a{1},b{2};queue={&a,&b};rw_camera_discard_queued();
 assert(queue.empty() && returned.size()==2 && returned[0]==&a && returned[1]==&b);
 rw_camera_discard_queued();assert(returned.size()==2);
}
'''
with tempfile.TemporaryDirectory() as tmp:
    path=Path(tmp);cpp=path/'cleanup.cpp';exe=path/'cleanup.exe';cpp.write_text(code)
    subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror',str(cpp),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print('Stopped queue cleanup: null/empty/full/repeated; only queue and return APIs PASS')
