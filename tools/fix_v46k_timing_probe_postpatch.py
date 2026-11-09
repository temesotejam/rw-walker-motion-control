from pathlib import Path

path = Path("src/experiment_runner.cpp")
text = path.read_text(encoding="utf-8")

if "r.gyro_sequence != timing_probe_event_.gyro_sequence_at_start" not in text:
    old = "  if (r.gyro_sequence != 0 && r.gyro_sequence != last_imu_update_us_) {\n    updateFilterSeries(r);\n"
    new = (
        "  if (r.gyro_sequence != 0 && r.gyro_sequence != last_imu_update_us_) {\n"
        "    if (timing_probe_pending_ && !timing_probe_imu_captured_ &&\n"
        "        r.gyro_sequence != timing_probe_event_.gyro_sequence_at_start) {\n"
        "      timing_probe_event_.first_imu_dt_after_start_us = r.gyro_update_dt_us;\n"
        "      timing_probe_event_.first_imu_sample_offset_us = r.last_gyro_update_us == 0 ? 0 :\n"
        "          static_cast<uint32_t>(r.last_gyro_update_us - timing_probe_event_.pulse_start_us);\n"
        "      timing_probe_imu_captured_ = true;\n"
        "      maybeFinalizeTimingProbe();\n"
        "    }\n"
        "    updateFilterSeries(r);\n"
    )
    if text.count(old) != 1:
        raise RuntimeError("fresh gyro insertion marker changed")
    text = text.replace(old, new, 1)

reset_token = "  timing_probe_event_ = PsramLogger::TimingProbeEvent{};\n"
if reset_token not in text:
    old = "void ExperimentRunner::resetEnergyControlAutonomous() {\n"
    new = (
        old
        + "  timing_probe_event_ = PsramLogger::TimingProbeEvent{};\n"
        + "  timing_probe_pending_ = false;\n"
        + "  timing_probe_loop_captured_ = false;\n"
        + "  timing_probe_log_captured_ = false;\n"
        + "  timing_probe_imu_captured_ = false;\n"
    )
    if text.count(old) != 1:
        raise RuntimeError("resetEnergyControlAutonomous marker changed")
    text = text.replace(old, new, 1)

path.write_text(text, encoding="utf-8")
print("Applied V46k post-patch timing inserts")
