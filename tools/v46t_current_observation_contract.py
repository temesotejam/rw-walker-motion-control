"""Exact inverse of the logging-only V46t change, for full-controller hash guards."""
OLD_PREFIX = "  const uint32_t now_us = micros();\n  if (logger_->full()) {"
NEW_PREFIX = """  // V46t: copy telemetry before taking its reference clock. Never re-read it
  // while constructing this row; Core0 may publish between any two statements.
  const RollerTelemetry roller_telemetry = roller_->telemetrySnapshot();
  const uint32_t now_us = micros();
  if (logger_->full()) {"""
SNAPSHOT = "  const RollerTelemetry roller_telemetry = roller_->telemetrySnapshot();\n"
CURRENT = "  row.roller_actual_current_mA = roller_telemetry.actual_current_mA;"
OLD_AGE = "  row.roller_current_age_us = roller_->currentAgeUs(now_us);"
NEW_AGE = """  row.roller_current_age_us = roller_telemetry.current_sample_time_us == 0
      ? UINT32_MAX : static_cast<uint32_t>(now_us - roller_telemetry.current_sample_time_us);"""

def normalize_current_observation(text):
    if '// V46t: copy telemetry' not in text:
        return text
    start=text.index('void ExperimentRunner::logSampleNow() {')
    prefix, body=text[:start], text[start:]
    for old,new in ((NEW_PREFIX,OLD_PREFIX),(CURRENT,SNAPSHOT+CURRENT),(NEW_AGE,OLD_AGE)):
        if body.count(old)!=1:
            raise ValueError('V46t observation structure changed; review required')
        body=body.replace(old,new,1)
    return prefix+body
