"""Invert only the documented V46af side gate for historical hash checks.

The live side gate is exercised in test_v46ae_mekf_amplitude.py, including the
zero-output, projected-before-posterior crossing that missed peaks in d170.
"""

SIDE_GATE = """  // V46af: a compensated zero-cross can precede the posterior zero-cross.
  // With no output, tracking resumes immediately and may still see the old
  // side. Wait for the commanded half-cycle before seeding its extremum.
  if (energy_control_autonomous_pending_peak_ &&
      detector_side != energy_control_autonomous_pending_next_side_) return;
"""


def normalize_v46af(text: str, path: str) -> str:
    from v46ag_rate_baseline_contract import normalize_v46ag
    text = normalize_v46ag(text, path)
    if path == "src/experiment_runner.cpp":
        anchor = "  if (detector_side == 0) return;\n"
        old = anchor + "  const float detector_abs_deg = fabsf(peak_relative_angle_deg);\n"
        new = anchor + SIDE_GATE + "  const float detector_abs_deg = fabsf(peak_relative_angle_deg);\n"
        if text.count(new) == 1:
            text = text.replace(new, old, 1)
        elif text.count(old) != 1:
            raise ValueError("V46af peak-side delta changed")
    if path == "src/config.h":
        old = '"v46ae_mekf_amplitude_20260919"'
        new = '"v46af_mekf_peak_side_20260919"'
        if text.count(new) == 1:
            text = text.replace(new, old, 1)
        elif text.count(old) != 1:
            raise ValueError("V46af revision delta changed")
    return text
