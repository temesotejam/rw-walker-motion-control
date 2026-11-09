#!/usr/bin/env python3
"""One-time integration of bounded IMU diagnostics into the existing large files.
No motor selector, output predicate, estimator, or controller constant is edited.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def replace(path, old, new):
    p = ROOT / path
    s = p.read_text(encoding='utf-8')
    if new in s:
        return
    if s.count(old) != 1:
        raise RuntimeError(f'{path}: expected one anchor, got {s.count(old)}')
    p.write_text(s.replace(old, new, 1), encoding='utf-8')

replace('src/imu_acquisition_audit.h',
    '      if (gap_count < kGapCapacity) gaps[gap_count++] = {relative_us, dt_us, sequence};',
    '      if (gap_count < kGapCapacity) {\n        Gap& g = gaps[gap_count++];\n        g.time_us = relative_us; g.dt_us = dt_us; g.sequence = sequence;\n      }')
replace('src/experiment_runner.cpp',
    'r.gyro_sequence != timing_probe_event_.gyro_sequence_at_start) {',
    'r.gyro_sequence != timing_probe_event_.gyro_sequence_at_start &&\n        static_cast<int32_t>(r.last_gyro_update_us - timing_probe_event_.pulse_start_us) >= 0) {')
replace('src/psram_logger.cpp', '#include "psram_logger.h"',
    '#include "psram_logger.h"\n#include "imu_manager.h"\nextern ImuManager imu;')
anchor = '  json += "\\"v46l_solver_shadow_revision\\":\\"v46l_discrete_ternary_shadow_20260914\\",";'
replace('src/psram_logger.cpp', anchor,
    '  json += "\\"v46n_imu_acquisition\\":" + imu.acquisitionDiagnosticsJson() + ",";\n' + anchor)
replace('src/web_ui.cpp', '  server_->on("/status.json", HTTP_GET, [this]() { handleStatus(); });',
    '  server_->on("/status.json", HTTP_GET, [this]() { handleStatus(); });\n'
    '  server_->on("/imu-acquisition.json", HTTP_GET, [this]() {\n'
    '    if (runner_->running()) { server_->send(409, "text/plain", "read_after_run"); return; }\n'
    '    server_->sendHeader("Cache-Control", "no-store");\n'
    '    server_->send(200, "application/json", imu_->acquisitionDiagnosticsJson());\n'
    '  });')
# An explicit acquisition label, separate from the frozen V46l attitude revision.
replace('src/web_ui.cpp', '<header><h1>Autonomous Energy Control V7</h1></header>',
    '<header><h1>Autonomous Energy Control V7</h1><div>V46n / 0.46.13 / priority IMU acquisition</div></header>')

# Only release-identity expectations change in inherited tests; safety assertions remain.
for path in ('tools/test_v46i_task_split_source_guards.py', 'tools/test_v46l_fast_solver_shadow.py'):
    p = ROOT / path
    s = p.read_text(encoding='utf-8')
    s = s.replace('"version": "0.46.11"', '"version": "0.46.13"')
    if path.endswith('test_v46i_task_split_source_guards.py'):
        s = s.replace("assert 'AtomS3R V46l MEKF Motor Validation' in manifest",
                      "assert 'AtomS3R V46n MEKF Motor Validation' in manifest")
    p.write_text(s, encoding='utf-8')

(ROOT / 'site/manifest.json').write_text('''{
  "name": "AtomS3R V46n MEKF Motor Validation",
  "version": "0.46.13",
  "builds": [{"chipFamily": "ESP32-S3", "parts": [
    {"path": "firmware/bootloader.bin", "offset": 0},
    {"path": "firmware/partitions.bin", "offset": 32768},
    {"path": "firmware/boot_app0.bin", "offset": 57344},
    {"path": "firmware/firmware.bin", "offset": 65536}
  ]}]
}
''', encoding='utf-8')
p = ROOT / 'site/index.html'
s = p.read_text(encoding='utf-8').replace('V46l', 'V46n')
if 'v46n_priority_imu_20260914' not in s:
    s = s.replace('    <section class="panel important">', '''    <section class="panel important">
      <h2>V46n / 0.46.13：モータ駆動中のIMU取得を独立化</h2>
      <p>取得版：<code>v46n_priority_imu_20260914</code>。従来のV7モータ制御、
      ±300 mA・最大100 ms、立位判定、非常停止、2 ms電流監査は維持しています。</p>
      <p>Core 1の優先度6タスクがBMI270を取得し、32件の時刻付きキューから
      MEKF・従来solverへ順番に渡します。Roller通信はCore 0のままです。
      キューあふれ、受け渡し遅れ10 ms超、取得停止は異常として停止します。</p>
      <p>RWLOGの<code>v46n_imu_acquisition</code>には全取得サンプルの周期集計と
      受け渡し遅れを別々に保存します。通常RWLOG行は全IMUサンプルではありません。
      モータ駆動中の改善量は実機Runで確認してください。</p>
      <p>姿勢推定・制御の基準版はV46lのままです。今回、solverやピーク振幅の定義は変更していません。</p>
    </section>

    <section class="panel important">''', 1)
p.write_text(s, encoding='utf-8')

build = '''name: build
on:
  push:
    branches: [main, "feature/**"]
  pull_request:
permissions:
  contents: read
concurrency:
  group: build-${{ github.ref }}
  cancel-in-progress: true
jobs:
  platformio:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0
      - uses: actions/setup-python@v5
        with:
          python-version: "3.11"
      - name: Verify retained baseline and IMU ownership
        run: python tools/test_v46n_acquisition_source.py
      - name: Run native acquisition accounting tests
        run: |
          g++ -std=c++11 -O2 tools/test_v46n_acquisition.cpp -o /tmp/test_acq
          /tmp/test_acq
      - name: Run all inherited control and format guards
        run: |
          python tools/test_current_audit_source_guards.py
          python tools/test_v46_motor_validation_source_guards.py
          python tools/test_v46g_highrate_source_guards.py
          python tools/test_v46i_task_split_source_guards.py
          python tools/test_v46k_timing_probe_source_guards.py
          python tools/test_v46l_fast_solver_shadow.py
          python tools/test_rwlog_v46_converter.py
          g++ -std=c++17 -O2 tools/test_mekf_host.cpp src/mekf6.cpp -o /tmp/test_mekf
          /tmp/test_mekf
      - name: Install PlatformIO
        run: pip install platformio
      - name: Build motor-driven V46n firmware
        run: pio run -e atoms3cam
      - name: Package flash images
        run: |
          set -euxo pipefail
          mkdir -p dist
          cp .pio/build/atoms3cam/{bootloader.bin,partitions.bin,firmware.bin,firmware.elf} dist/
          cp "$HOME/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin" dist/
          python "$HOME/.platformio/packages/tool-esptoolpy/esptool.py" --chip esp32s3 merge_bin -o dist/merged-firmware.bin 0x0000 dist/bootloader.bin 0x8000 dist/partitions.bin 0xe000 dist/boot_app0.bin 0x10000 dist/firmware.bin
          printf '%s\\n' "$GITHUB_SHA" > dist/build-sha.txt
          printf '%s\\n' 'V46n 0.46.13; merged-firmware.bin at 0x0000; real V7 motor controller retained' > dist/FLASH_LAYOUT.txt
          (cd dist && sha256sum *.bin > SHA256SUMS.txt)
          git diff --exit-code
      - uses: actions/upload-artifact@v4
        with:
          name: atoms3r-v46n-${{ github.sha }}
          path: dist/
          if-no-files-found: error
          retention-days: 30
'''
(ROOT / '.github/workflows/build.yml').write_text(build, encoding='utf-8')

pages = '''name: pages
on:
  push:
    branches: [main]
  workflow_dispatch:
permissions:
  contents: read
  pages: write
  id-token: write
concurrency:
  group: pages
  cancel-in-progress: true
jobs:
  build-site:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0
      - uses: actions/setup-python@v5
        with:
          python-version: "3.11"
      - name: Verify V46n release and retained motor guards
        run: |
          python tools/test_v46n_acquisition_source.py
          python tools/test_current_audit_source_guards.py
          python tools/test_v46_motor_validation_source_guards.py
          python tools/test_v46g_highrate_source_guards.py
          python tools/test_v46i_task_split_source_guards.py
          python tools/test_v46k_timing_probe_source_guards.py
          python tools/test_v46l_fast_solver_shadow.py
          python tools/test_rwlog_v46_converter.py
          g++ -std=c++11 -O2 tools/test_v46n_acquisition.cpp -o /tmp/test_acq
          /tmp/test_acq
          python -m json.tool site/manifest.json >/dev/null
          grep -q 'v46n_priority_imu_20260914' site/index.html
      - name: Install PlatformIO
        run: pip install platformio
      - name: Build V46n motor-driven firmware
        run: pio run -e atoms3cam
      - name: Assemble verified web flasher
        run: |
          set -euxo pipefail
          mkdir -p _site/firmware
          cp -R site/. _site/
          cp .pio/build/atoms3cam/{bootloader.bin,partitions.bin,firmware.bin} _site/firmware/
          cp "$HOME/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin" _site/firmware/
          printf '%s\\n' "$GITHUB_SHA" > _site/build-sha.txt
          (cd _site/firmware && sha256sum *.bin > SHA256SUMS.txt)
          git diff --exit-code
      - uses: actions/configure-pages@v5
        with:
          enablement: true
      - uses: actions/upload-pages-artifact@v3
        with:
          path: _site
  deploy:
    needs: build-site
    runs-on: ubuntu-latest
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    steps:
      - uses: actions/deploy-pages@v4
        id: deployment
'''
(ROOT / '.github/workflows/pages.yml').write_text(pages, encoding='utf-8')
print('V46n release files integrated; no control equations or output gates changed')
