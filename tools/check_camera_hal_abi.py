"""Fail closed if the binary camera ABI differs from the pinned v21 source."""
import hashlib
from pathlib import Path

EXPECTED = '3d33f69109171788fcec92148499bb5766e348dbdbe79aa50bfb19180cbde716'


def verify(framework):
    archive = Path(framework) / 'tools/sdk/esp32s3/lib/libesp32-camera.a'
    actual = hashlib.sha256(archive.read_bytes()).hexdigest()
    if actual != EXPECTED:
        raise RuntimeError('v21 camera HAL ABI not verified for SDK archive ' + actual)
    print('v21 pinned ESP32 camera archive ABI fingerprint verified')


if 'Import' in globals():
    Import('env')
    verify(env.PioPlatform().get_package_dir('framework-arduinoespressif32'))
