"""Exercise actual angle conversion, bounded operating range, Start and CSV."""
from pathlib import Path
import csv
import json
import subprocess
import tempfile
import convert_rwlog_to_csv as converter

ROOT=Path(__file__).resolve().parents[1]
CPP=r'''
#include "foot_support_start.h"
#include "foot_angle_estimator.h"
#include "foot_tracking_config.h"
#include "foot_range_diagnostics.h"
#include <cassert>
#include <iostream>
struct Frame {bool zero_ready=true,frame_valid=true,timestamp_valid=true,left_valid=true,right_valid=true,left_in_range=true,right_in_range=true;
 uint64_t frame_us=900000;float left_deg=0,right_deg=0,left_x=201,right_x=203;};
struct Snapshot {bool available=true,zero_ready=true;Frame latest;};
int main(){
 for(bool right:{false,true})for(bool low:{false,true})for(float extra:{0.f,.1f,1.f,3.f,5.f,5.02f,8.f,20.f}){
   const auto* p=right?foot_calibration::right:foot_calibration::left;
   const unsigned i=low?0:foot_calibration::count-2;
   const auto edge=low?p[0]:p[foot_calibration::count-1];
   const float slope=(p[i+1].angle-p[i].angle)/(p[i+1].x-p[i].x);
   const float x=edge.x+(low?extra:-extra)/slope;
   const auto q=foot_control_range::assess(x,right);
   assert(q.accepted==(extra<=5.f));assert(std::fabs(q.extension_deg-extra)<.00002f);
   assert(q.measured==(extra==0));
   WhiteMarkerObservation marker;marker.valid=true;marker.id=right?appcfg::kMarkerAId:appcfg::kMarkerBId;
   marker.center_x_px=x;
   const auto a=estimateFootAngle(marker,p[foot_calibration::count-1].x,true);
   assert(a.valid && a.in_calibration_range==q.measured);
   assert(std::fabs(a.angle_deg-(edge.angle+(low?extra:-extra)))<.00003f); // no angle clipping
   Snapshot s;
   if(right){s.latest.right_x=x;s.latest.right_deg=a.angle_deg;s.latest.right_in_range=q.measured;}
   else{s.latest.left_x=x;s.latest.left_deg=a.angle_deg;s.latest.left_in_range=q.measured;}
   assert(foot_support_start::ready(s,1000000)==q.accepted);
   s.zero_ready=false;assert(!foot_support_start::ready(s,1000000));s.zero_ready=true;
   assert(!foot_support_start::ready(s,1100001));assert(!foot_support_start::ready(s,899999));
   s.latest.left_valid=false;assert(!foot_support_start::ready(s,1000000));
 }
 assert(!foot_control_range::assess(NAN,true).accepted);
 assert(!foot_control_range::assess(INFINITY,false).accepted);
 std::cout<<"range tolerance PASS: both feet/both ends, 3/5deg accepted, 5.02/8/20deg rejected, original angles and other Start gates preserved\n";
 std::cout<<footRangeDiagnosticsJson().c_str()<<'\n';
}
'''
with tempfile.TemporaryDirectory(prefix='foot_control_range_') as folder:
    p=Path(folder)
    (p/'test.cpp').write_text(CPP,encoding='utf8')
    subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-DWALKING_RETURN_ENERGY=1',
                    '-I'+str(ROOT/'src'),'-I'+str(ROOT/'tools/host_v46o'),str(p/'test.cpp'),str(ROOT/'src/foot_angle_estimator.cpp'),'-o',str(p/'test.exe')],check=True)
    output=subprocess.check_output([str(p/'test.exe')],text=True).splitlines()
    print(output[0]);diagnostics=json.loads(output[1])
    assert diagnostics['control_extension_deg']==5 and diagnostics['extension_angle_accuracy_validated'] is False
    assert 'in_range_flags_remain_measured_support' in diagnostics['control_extension_policy']
    for expanded in [False,True]:
        out=p/str(expanded);out.mkdir()
        row={'left_in_range':False,'right_in_range':True}
        if expanded:row.update(left_control_range_ok=True,right_control_range_ok=True,left_extrapolation_deg=3,right_extrapolation_deg=0)
        assert converter.write_foot_frames({'foot_frames':[row]},out)==1
        with (out/'foot_angles.csv').open(encoding='utf8',newline='') as stream:rows=list(csv.DictReader(stream))
        if expanded:
            assert rows[0]['left_in_range']=='False' and rows[0]['left_control_range_ok']=='True'
            assert float(rows[0]['left_extrapolation_deg'])==3
        else:assert list(rows[0])==converter.FOOT_COLUMNS
print('range CSV PASS: measured support kept distinct, optional operating-range fields preserved, old columns unchanged')
