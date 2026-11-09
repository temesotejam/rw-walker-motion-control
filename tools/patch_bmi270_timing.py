#!/usr/bin/env python3
"""Auditable build-local patch of exactly M5Unified 0.2.18, never silent fallback.

PlatformIO runs this as a pre script after `pio pkg install`. No other library
method is modified. The source and emitted function are hash/structure checked
on every build; the library header is refreshed from the versioned project copy.
"""
from pathlib import Path
import hashlib
import json

UPSTREAM_BLOB = '41732382bbdfe413be758748bca24d9a15a3a1d8'
BEGIN = '  IMU_Base::imu_spec_t BMI270_Class::getImuRawData('
END = '  void BMI270_Class::getConvertParam('
INCLUDE = '#include "BMI270_TimingReader_v46u.hpp"\n#include <esp_timer.h>\n'
DELEGATE = '''  IMU_Base::imu_spec_t BMI270_Class::getImuRawData(imu_raw_data_t* data) const
  {
    return static_cast<imu_spec_t>(bmi270_timing::readRaw(data,
      [this](uint8_t reg, uint8_t* dst, size_t n) {
        const auto transport = bmi270_timing::transport();
        return transport ? transport(reg, dst, n) : readRegister(reg, dst, n);
      }, []() { return static_cast<uint32_t>(esp_timer_get_time()); }));
  }

'''

def git_blob(raw):
    return hashlib.sha1(b'blob '+str(len(raw)).encode()+b'\0'+raw).hexdigest()

def patch(root, library):
    path = library/'src/utility/imu/BMI270_Class.cpp'
    backup = path.with_suffix('.cpp.v46u-original')
    raw = path.read_bytes()
    if backup.exists():
        original = backup.read_bytes()
    else:
        original = raw
    if git_blob(original) != UPSTREAM_BLOB:
        raise RuntimeError('BMI270 source is not the verified M5Unified 0.2.18; refusing to patch')
    text = original.decode()
    start = text.index(BEGIN); end = text.index(END, start)
    expected = (text[:start]+DELEGATE+text[end:]).replace(
        '#include "BMI270_Class.hpp"\n', '#include "BMI270_Class.hpp"\n'+INCLUDE, 1).encode()
    previous_verified = hashlib.sha256(raw).hexdigest() == '54bf8f7c9016033cee6015331ed65a35b56e74375040ba4994fbb06cee0b430b'
    if raw not in (original, expected) and not previous_verified:
        raise RuntimeError('BMI270 source has an unexpected local edit; refusing to overwrite')
    header = (root/'src/bmi270_timing_reader.h').read_bytes()
    if not backup.exists(): backup.write_bytes(original)
    if raw != expected: path.write_bytes(expected)
    (path.parent/'BMI270_TimingReader_v46u.hpp').write_bytes(header)
    manifest = {'upstream_git_blob': UPSTREAM_BLOB,
                'patched_sha256': hashlib.sha256(expected).hexdigest(),
                'header_sha256': hashlib.sha256(header).hexdigest(),
                'policy': 'STATUS_0x03_selective_DATA_reads;boot_M5GFX_then_exclusive_IDF_interrupt_I2C1;unchanged_conversion_ODR_filters'}
    (root/'bmi270-build-patch.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('V46u BMI270 transport patch verified:', manifest['patched_sha256'])
    return manifest

if __name__ == '__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--library',type=Path,required=True)
    args=parser.parse_args();patch(Path(__file__).resolve().parents[1],args.library)
else:
    try:
        Import('env')
    except NameError:
        pass
    else:
        root=Path(env.subst('$PROJECT_DIR'))
        patch(root, Path(env.subst('$PROJECT_LIBDEPS_DIR'))/env.subst('$PIOENV')/'M5Unified')
