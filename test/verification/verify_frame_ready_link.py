"""Confirm v21 replaces only the HAL object, retaining the pinned low-level DMA."""
import argparse
from pathlib import Path
import re

def verify(path):
    text = path.read_text(encoding='utf8')
    assert 'libesp32-camera.a(cam_hal.c.obj)' not in text, 'SDK HAL was unexpectedly linked'
    for symbol in ['cam_init','cam_config','cam_take','cam_give','ll_cam_send_event','rw_camera_frame_ready_stats']:
        assert re.search(r'^'+symbol+r'\s+.*src/camera_frame_ready_hal.c.o$',text,re.M), symbol
    assert re.search(r'^ll_cam_stop\s+.*libesp32-camera.a\(ll_cam.c.obj\)$',text,re.M)
    print('PASS linked v21 HAL owns camera symbols; pinned SDK still supplies low-level DMA')

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('map',type=Path)
    verify(parser.parse_args().map)
