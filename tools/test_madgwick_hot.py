#!/usr/bin/env python3
"""Compare actual annotated AHRS production code with frozen upstream at -Os.
Verify both IMU overloads, lazy getters, and strict patch provenance. This is
arithmetic regression testing, not an ESP32 time or scheduling measurement.
"""
from pathlib import Path
import hashlib
import re
import shutil
import subprocess
import tempfile
from patch_madgwick_hot import ROOT, FIXTURES, HEADER, patch
from verify_madgwick_dependency import verify

# A comment-only policy edit changes its provenance hash too. Reject a stale
# live-publication pin during host verification, before building/deploying.
workflow = (ROOT / '.github/workflows/pages.yml').read_text()
policy_pins = re.findall(r"madgwick\['policy_header_sha256'\] != '([0-9a-f]{64})'", workflow)
assert policy_pins == [hashlib.sha256((ROOT / 'src/realtime_code.h').read_bytes()).hexdigest()], \
    'Live publication compiler-policy pin differs from the reviewed source header'

with tempfile.TemporaryDirectory() as directory:
    p = Path(directory)
    lib = p / 'candidate'
    shutil.copytree(FIXTURES, lib / 'src')
    (p / 'src').mkdir()
    shutil.copyfile(ROOT / 'src/realtime_code.h', p / 'src/realtime_code.h')
    first = patch(p, lib)
    assert patch(p, lib) == first
    verify(lib)
    original_cpp = (lib / 'src/Adafruit_AHRS_Madgwick.cpp').read_bytes()
    (lib / 'src/Adafruit_AHRS_Madgwick.cpp').write_bytes(original_cpp + b'\n// unexpected edit\n')
    try:
        patch(p, lib)
    except RuntimeError:
        pass
    else:
        raise AssertionError('Unexpected upstream source edit was accepted')
    (lib / 'src/Adafruit_AHRS_Madgwick.cpp').write_bytes(original_cpp)
    # Independent class compiled from the unmodified upstream arithmetic.
    reference = p / 'reference'
    shutil.copytree(FIXTURES, reference)
    for name in (HEADER, 'Adafruit_AHRS_Madgwick.cpp'):
        f = reference / name
        f.write_text(f.read_text().replace('Adafruit_Madgwick', 'ReferenceMadgwick')
                     .replace('__Adafruit_Madgwick_h__', '__ReferenceMadgwick_h__'))
    cpp = r'''
#include <cassert>
#include <cmath>
#include <cstdio>
#include "reference/Adafruit_AHRS_Madgwick.h"
#include "candidate/src/Adafruit_AHRS_Madgwick.h"
static void same(float a, float b) {
  assert((std::isnan(a) && std::isnan(b)) || (a==b && std::signbit(a)==std::signbit(b)));
}
int main() {
  unsigned updates=0;
  for(unsigned stream=0;stream<6;++stream) {
    ReferenceMadgwick old; Adafruit_Madgwick now;
    old.begin(200); now.begin(200);
    for(unsigned n=0;n<12000;++n) {
      const float t=n*0.0025f;
      float gx=10*sinf(t*2), gy=120*cosf(t*7), gz=15*sinf(t*3);
      float ax=.25f*sinf(t*7),ay=.1f*cosf(t*5),az=-1;
      if(stream==1 && n%3==0) ax=ay=az=0;  // no accel correction
      if(stream==2) { ax*=4;ay*=4;az*=4; } // normalization under vibration
      if(stream==3) { gx=gy=gz=0;ax=ay=0;az=1; } // zero gradient
      if(stream==4) { gx*=12;gy*=12;gz*=12; } // rapid multi-axis motion
      if(stream==5 && n==6000) ax=NAN;     // same propagation; no finite-only math
      const float beta=n%137<31 ? 0 : (n%137)/1370.0f;
      old.setBeta(beta);now.setBeta(beta);
      if(n%2) {
        old.updateIMU(gx,gy,gz,ax,ay,az);
        now.updateIMU(gx,gy,gz,ax,ay,az);
      } else {
        const float dt=.002f+(n%11)*.0002f;
        old.updateIMU(gx,gy,gz,ax,ay,az,dt);
        now.updateIMU(gx,gy,gz,ax,ay,az,dt);
      }
      float a[4],b[4];old.getQuaternion(a,a+1,a+2,a+3);now.getQuaternion(b,b+1,b+2,b+3);
      for(unsigned j=0;j<4;++j) same(a[j],b[j]);
      same(old.getPitch(),now.getPitch());same(old.getRoll(),now.getRoll());same(old.getYaw(),now.getYaw());
      old.getGravityVector(a,a+1,a+2);now.getGravityVector(b,b+1,b+2);
      for(unsigned j=0;j<3;++j) same(a[j],b[j]);
      ++updates;
    }
  }
  printf("Madgwick O2/no-fast-math vs upstream Os: %u updates exact; both overloads/all-axis getters/provenance PASS\n", updates);
}
'''
    (p / 'test.cpp').write_text(cpp)
    subprocess.run(['g++', '-std=c++17', '-Os', '-ffp-contract=off', '-Wall', '-Wextra', '-Werror',
                    str(p / 'test.cpp'), str(reference / 'Adafruit_AHRS_Madgwick.cpp'),
                    str(lib / 'src/Adafruit_AHRS_Madgwick.cpp'), '-o', str(p / 'test')], check=True)
    subprocess.run([str(p / 'test')], check=True)
