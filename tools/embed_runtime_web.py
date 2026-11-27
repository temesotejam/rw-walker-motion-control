#!/usr/bin/env python3
"""Generate the embedded measurement page from web/ during the firmware build."""
import argparse
import gzip
import io
from pathlib import Path
import sys


def render(root):
    web = root / 'web'
    html = (web / 'index.html').read_text(encoding='utf-8')
    for marker, filename in [('POSE_SCRIPT', 'pose_comparison.js'),
                             ('WHEEL_WORK_SCRIPT', 'wheel_work.js'), ('RUNTIME_SCRIPT', 'runtime.js')]:
        slot = f'/*{marker}*/'
        if html.count(slot) != 1:
            raise ValueError(f'Expected one {slot} in the measurement page')
        html = html.replace(slot, (web / filename).read_text(encoding='utf-8'))
    source = html.encode('utf-8')
    compressed = io.BytesIO()
    with gzip.GzipFile(fileobj=compressed, filename='', mode='wb', compresslevel=9, mtime=0) as archive:
        archive.write(source)
    payload = compressed.getvalue()
    literals = '\n'.join('"' + ''.join(f'\\x{byte:02x}' for byte in payload[i:i+32]) + '"'
                         for i in range(0, len(payload), 32))
    return ('#pragma once\n#include <Arduino.h>\nstatic const char RUNTIME_HTML[] PROGMEM = R"FREEFOOT('
            + html + ')FREEFOOT";\nstatic const char RUNTIME_HTML_GZIP[] PROGMEM =\n' + literals + ';\n').encode('utf-8')


def generate(root, directory):
    path = directory / 'freefoot_runtime_web.h'
    data = render(root)
    if not path.is_file() or path.read_bytes() != data:
        directory.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)


if 'Import' in globals():
    Import('env')
    include = Path(env.subst('$BUILD_DIR')) / 'generated'
    generate(Path(env.subst('$PROJECT_DIR')), include)
    env.Append(CPPPATH=[str(include)])
elif __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--stdout', action='store_true', help='write the generated header to stdout')
    mode.add_argument('--check', action='store_true', help='validate sources and any existing normal-build header')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    directory = root / '.pio/build/atoms3cam-foot-periodic40/generated'
    if args.stdout:
        sys.stdout.buffer.write(render(root))
    elif args.check:
        header = directory / 'freefoot_runtime_web.h'
        expected = render(root)
        if header.is_file() and header.read_bytes() != expected:
            parser.exit(1, 'Build the firmware to refresh the generated page.\n')
    else:
        generate(root, directory)
