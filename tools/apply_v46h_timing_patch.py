from pathlib import Path

# Trigger marker: V46h timing migration pass 5.


def replace_once(path: str, old: str, new: str) -> None:
    p = Path(path)
    text = p.read_text(encoding="utf-8")
    if text.count(old) != 1:
        raise RuntimeError(f"{path}: expected exactly one match for {old!r}, got {text.count(old)}")
    p.write_text(text.replace(old, new, 1), encoding="utf-8")

# Poll the BMI270 data-ready state faster than the 400 Hz gyro ODR so loop
# quantization does not turn the nominal 2.5 ms stream into ~3 ms updates.
replace_once(
    "src/config.h",
    "static constexpr uint32_t IMU_POLL_PERIOD_US = 2500UL;",
    "static constexpr uint32_t IMU_POLL_PERIOD_US = 1000UL;",
)
replace_once(
    "src/config.h",
    'static constexpr char ATTITUDE_VALIDATION_REVISION[] = "v46g_mekf_400hz_predict_200hz_accel_20260913";',
    'static constexpr char ATTITUDE_VALIDATION_REVISION[] = "v46h_mekf_400hz_web_quiet_predict_20260913";',
)

# Run-start gravity averaging already uses fresh accel sequence in V46g; keep it.
p = Path("src/experiment_runner.cpp")
text = p.read_text(encoding="utf-8")
if "r.accel_sequence != g_v46_mekf_run_reinit.last_accel_sequence" not in text:
    text = text.replace("uint32_t last_imu_update_us = 0;", "uint32_t last_accel_sequence = 0;", 1)
    text = text.replace(
        "r.last_update_us != g_v46_mekf_run_reinit.last_imu_update_us) {\n      g_v46_mekf_run_reinit.last_imu_update_us = r.last_update_us;",
        "r.accel_sequence != 0 &&\n        r.accel_sequence != g_v46_mekf_run_reinit.last_accel_sequence) {\n      g_v46_mekf_run_reinit.last_accel_sequence = r.accel_sequence;",
        1,
    )
p.write_text(text, encoding="utf-8")

# Browser display remains frozen by design during the measurement. In V46g
# the JS still fetched the large /status.json every 500 ms. Freeze network
# status polling as well, while preserving the emergency-stop POST.
p = Path("src/web_ui.cpp")
text = p.read_text(encoding="utf-8")
text = text.replace(
    "async function postStop(){await post('/stop');}",
    "async function postStop(){displayFrozen=false;await post('/stop');}",
    1,
)
text = text.replace(
    "async function refresh(){if(refreshInFlight)return;refreshInFlight=true;",
    "async function refresh(){if(displayFrozen||refreshInFlight)return;refreshInFlight=true;",
    1,
)
text = text.replace(
    "if(lastStatus.running){displayFrozen=true;applyFrozenState();return;}",
    "if(lastStatus.running){displayFrozen=true;applyFrozenState();setTimeout(()=>{displayFrozen=false;refresh();},41000);return;}",
    1,
)
p.write_text(text, encoding="utf-8")

# Identity/UI files.
replace_once(
    "src/main.cpp",
    "AtomS3R V46g MEKF motor-driven dynamic validation",
    "AtomS3R V46h MEKF motor-driven dynamic validation",
)
replace_once("src/main.cpp", "V46g identity:", "V46h identity:")
replace_once("src/main.cpp", 'displayLine("V46g MEKF",', 'displayLine("V46h MEKF",')

p = Path("site/index.html")
s = p.read_text(encoding="utf-8").replace("V46g", "V46h")
s = s.replace("v46g_mekf_400hz_predict_200hz_accel_20260913", "v46h_mekf_400hz_web_quiet_predict_20260913")
p.write_text(s, encoding="utf-8")

p = Path("site/manifest.json")
s = p.read_text(encoding="utf-8").replace("V46g", "V46h").replace('"version": "0.46.6"', '"version": "0.46.7"')
p.write_text(s, encoding="utf-8")

# Guard the timing fix itself.
guard = Path("tools/test_v46g_highrate_source_guards.py")
s = guard.read_text(encoding="utf-8")
s = s.replace("v46g_mekf_400hz_predict_200hz_accel_20260913", "v46h_mekf_400hz_web_quiet_predict_20260913")
s = s.replace("IMU_POLL_PERIOD_US = 2500UL", "IMU_POLL_PERIOD_US = 1000UL")
if "displayFrozen||refreshInFlight" not in s:
    s += '\nweb = Path("src/web_ui.cpp").read_text(encoding="utf-8")\nassert "displayFrozen||refreshInFlight" in web\nassert "setTimeout(()=>{displayFrozen=false;refresh();},41000)" in web\n'
guard.write_text(s, encoding="utf-8")

print("Applied V46h web-quiet / fast-poll timing patch")
