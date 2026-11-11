"""Portable geometry/prediction and compact metadata conversion checks."""
import importlib.util
from pathlib import Path
import subprocess
import tempfile
import csv

root=Path(__file__).resolve().parents[1]
source=r'''
#include "cad_support.h"
#include "foot_pose_prediction.h"
#include <cassert>
#include <cmath>
int main(){
 float a;assert(cad_support::switchAngle(0,0,0,a) && std::fabs(a)<.001f);
 assert(cad_support::switchAngle(0,2,0,a) && std::fabs(a+4.416f)<.005f);
 assert(cad_support::switchAngle(-.80957f,3.74576f,0,a) && std::fabs(a+5.24638f)<.005f);
 assert(!cad_support::switchAngle(NAN,0,0,a));
 using namespace foot_pose_prediction;
 Predictor p;assert(p.observe(1,100,150,true,0,1).reason==Reason::Warmup);
 auto q=p.observe(1,200,250,true,1,3);
 assert(q.ready() && std::fabs(q.left-1.8f)<.001f && std::fabs(q.right-4.6f)<.001f);
 assert(q.age_ms==50 && q.horizon_ms==80);
 assert(p.observe(1,300,350,false,2,4).reason==Reason::Invalid);
 assert(p.observe(1,400,450,true,3,5).reason==Reason::Warmup);
 assert(p.observe(2,500,550,true,4,6).reason==Reason::Warmup);
 assert(p.observe(2,500,550,true,4,6).reason==Reason::Timestamp);
 assert(p.observe(2,600,550,true,4,6).reason==Reason::Timestamp);
 p.reset();p.observe(1,100,150,true,0,0);
 assert(p.observe(1,400,450,true,1,1).reason==Reason::Gap);
 assert(p.observe(1,500,550,true,20,0).reason==Reason::Jump);
 assert(p.observe(1,600,650,true,1,1).reason==Reason::Warmup);
 assert(p.observe(1,700,881,true,1,1).reason==Reason::Stale);
 assert(p.observe(1,900,1080,true,1,1).reason==Reason::Horizon);
}
'''
with tempfile.TemporaryDirectory(prefix='cad_shadow_') as td:
    td=Path(td);cpp=td/'test.cpp';exe=td/'test.exe';cpp.write_text(source)
    subprocess.run(['g++','-std=c++11','-O2','-Wall','-Wextra','-Werror','-I'+str(root/'src'),str(cpp),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
    spec=importlib.util.spec_from_file_location('converter',root/'tools/convert_rwlog_to_csv.py')
    converter=importlib.util.module_from_spec(spec);spec.loader.exec_module(converter)
    raw={'foot_frames':[{'sequence':4,'run_id':1,'frame_us':100}],
         'cad_support_shadow':{'output_enabled':False,'records':{'fields':['sequence','reason','predicted_left_deg'],'rows':[[4,0,1.2]]}}}
    meta=converter.expand_tables(raw)
    assert converter.write_cad_support_shadow(meta,td)==1
    with (td/'cad_support_shadow.csv').open() as f:row=next(csv.DictReader(f))
    assert row['frame_us']=='100' and row['predicted_left_deg']=='1.2'
    meta['cad_support_shadow']['records'].append({'sequence':4})
    try:converter.write_cad_support_shadow(meta,td)
    except ValueError:pass
    else:raise AssertionError('duplicate sequence accepted')
print('CAD geometry, prediction invalidation, causal timestamps and converter PASS')
