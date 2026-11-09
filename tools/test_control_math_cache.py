#!/usr/bin/env python3
"""Test production current/tau functions and frozen legacy width-cache arithmetic.
The active direct inverse is covered by test_direct_q_solver.py.
"""
import json
import re
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
reference = json.loads((ROOT / "tools/fixtures/control_math_04716.json").read_text())
inputs = json.loads((ROOT / "tools/fixtures/control_inputs_04716.json").read_text())
source = (ROOT / "src/experiment_runner.cpp").read_text()
fields = inputs["float_fields"]
if isinstance(fields, str):
    fields = fields.split(",")


def method(name):
    start = source.index("float ExperimentRunner::" + name + "(")
    end = source.index("{", start) + 1
    depth = 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def without_audit(text):
    return re.sub(r"  // V46s audit begin\n.*?  // V46s audit end\n", "", text, flags=re.S)


cached_fixture = json.loads((ROOT / "tools/fixtures/control_math_04717.json").read_text())
assert cached_fixture["solver_tail"] == reference["solver_tail"]
for name in ("energyControlPotentialJ", "energyControlAutonomousCorrectedPrediction"):
    assert method(name) == reference["methods"][name]


def model(name, methods, block):
    # Replay recorded float32 coefficients without substituting host libm for
    # the ESP32 powf that generated them. Model caching is checked separately.
    block = without_audit(block)
    a = block.index("  const float fast_v =")
    b = block.index("  auto evaluate_width", a)
    block = block[:a] + """  const float fast_signed_target_current_mA = r.signed_target_current_mA;
  const float fast_tau_s = r.tau_s;
""" + block[b:]
    # Instrument only the test build to count actual charge/energy evaluations.
    block = block.replace("float q_mA_s = 0.0f;", "++computed; float q_mA_s = 0.0f;")
    tail = without_audit(reference["solver_tail"])
    tail = re.sub(r"    logger_->addEnergyControlAutonomousZeroCrossEvent\(event\);\n", "", tail)
    tail = tail.replace("    rearm_for_next_peak();\n", "").replace("return;", "return out;")
    return "struct " + name + """ {
  Input r{};
  mutable control_math::CurrentModelCache current_model_cache_;
  float energyControlAutonomousGainForSide(int8_t) const { return r.base_gain; }
  void energyControlAutonomousCorrectionParameters(int8_t, float* c, float* g) const {
    *c = r.correction_c; *g = r.correction_gain;
  }
""" + "\n".join(m.replace("ExperimentRunner::", "") for m in methods.values()) + """
  Result replay() {
    Result out;
    unsigned computed = 0;
    PsramLogger::EnergyControlAutonomousZeroCrossEvent event;
    event.i0_estimated_mA = r.i0_mA;
    event.free_next_peak_amplitude_deg = r.free_peak_deg;
    event.physical_next_peak_side = r.physical_side;
    event.target_energy_j = r.target_energy_j;
    event.target_peak_deg = r.target_peak_deg;
    event.passive_energy_j = r.passive_energy_j;
    event.q1_gain_deg_per_mA_s = r.base_gain;
    event.q_available_mA_s = r.q_available_mA_s;
    const float energy_control_autonomous_integral_plus_mA_s_ = r.integral_mA_s;
    const float energy_control_autonomous_integral_minus_mA_s_ = r.integral_mA_s;
""" + block + tail + """
    out.valid = true;
    out.ff = ff_width_ms; out.selected = selected_width_ms;
    out.candidate = selected_fast.width_ms;
    out.evaluations = fast_eval_count; out.computed = computed;
    out.upper = event.q_saturated_upper; out.lower = event.q_saturated_lower;
    out.values = {ff_q_mA_s, ff_energy_j, event.q_unclamped_mA_s,
        corrected_q_target_mA_s, corrected_target_prediction_deg, corrected_target_energy_j,
        selected_q_mA_s, selected_energy_j, selected_fast.q_mA_s,
        selected_fast.energy_j, selected_fast.error_j, zero_output_error_j};
    return out;
  }
};
"""


cpp = r"""#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include "config.h"
#include "psram_logger.h"
#include "control_math_cache.h"
struct Input {
  int8_t physical_side = 1, command_direction = 1;
""" + "\n".join("  float " + f + " = 0;" for f in fields) + r"""
};
struct Result {
  bool valid = false, upper = false, lower = false;
  unsigned ff = 0, selected = 0, candidate = 0, evaluations = 0, computed = 0;
  std::array<float, 12> values{};
};
static uint32_t bits(float f) { uint32_t u; memcpy(&u,&f,4); return u; }
static float scalar(uint32_t u) { float f; memcpy(&f,&u,4); return f; }
static void same(float a, float b) {
  assert(bits(a) == bits(b) || (std::isnan(a) && std::isnan(b)));
}
"""
cpp += model("Reference", reference["methods"], reference["solver_block"])
cpp += model("Cached", {name: method(name) for name in reference["methods"]}, cached_fixture["solver_block"])
cpp += r"""
static Result compare(const Input& input) {
  Reference old; Cached now; old.r = now.r = input;
  const auto a = old.replay(), b = now.replay();
  assert(a.valid == b.valid && a.ff == b.ff && a.selected == b.selected);
  assert(a.candidate == b.candidate && a.evaluations == b.evaluations);
  assert(a.upper == b.upper && a.lower == b.lower);
  for (unsigned i=0;i<a.values.size();++i) same(a.values[i],b.values[i]);
  assert(b.computed <= a.computed);
  return b;
}
static uint32_t seed = 71623;
static float noise() {
  seed=1664525U*seed+1013904223U;
  return static_cast<float>(seed>>8)/8388608.0f-1.0f;
}
static void modelInputs() {
  Reference old; Cached now;
  // Repeated calls, sign changes, zero, nonfinite values and changes as small
  // as one float32 ULP must preserve the old current/tau expressions.
  const float nan=std::numeric_limits<float>::quiet_NaN();
  const float inf=std::numeric_limits<float>::infinity();
  const float commands[]={0,-0.0f,1,-1,300,300,-300,301,299,800,1000,nan,inf};
  const float voltages[]={0,-0.0f,4.0f,4.0f,std::nextafter(4.0f,5.0f),
      3.8f,4.2f,3.3f,5.0f,nan,inf};
  unsigned count=0;
  for (float u: commands) for(float v: voltages) for(int i=0;i<4;++i) {
    same(old.predictCurrentGoalMa(u,v),now.predictCurrentGoalMa(u,v));
    same(old.predictRiseTauS(u),now.predictRiseTauS(u)); ++count;
  }
  for(unsigned i=0;i<20000;++i) {
    const float u=500.0f*noise(), v=4.0f+noise();
    for(int j=0;j<4;++j) {
      same(old.predictCurrentGoalMa(u,v),now.predictCurrentGoalMa(u,v));
      same(old.predictRiseTauS(u),now.predictRiseTauS(u)); ++count;
    }
  }
  // Observe misses with a callback: voltage changes must not reuse a rounded
  // key, and fresh battery voltage must invalidate only the goal, not tau.
  control_math::CurrentModelCache cache;
  unsigned goals=0, taus=0;
  auto goal=[&](float u,float v) {return cache.goal(u,v,[&](){++goals; return u+v;});};
  auto tau=[&](float u) {return cache.riseTime(u,[&](){++taus; return u;});};
  for(int i=0;i<4;++i) {goal(300,4.0f);tau(300);}
  assert(goals==1 && taus==1);
  goal(300,std::nextafter(4.0f,5.0f));tau(300);
  assert(goals==2 && taus==1);
  goal(301,4.0f);tau(301);assert(goals==3 && taus==2);
  // Width cache handles all bit boundaries, invalid results and out-of-bound
  // requests without indexing outside storage. A new decision starts empty.
  for(int decision=0;decision<2;++decision) {
    control_math::WidthPredictionCache<100> widths;
    unsigned computed=0;
    for(int repeat=0;repeat<3;++repeat) for(uint16_t w=0;w<=100;++w) {
      auto p=widths.get(w,[&](){++computed; return control_math::WidthPrediction{float(w),nan};});
      same(p.q_mA_s,float(w)); assert(std::isnan(p.energy_j));
    }
    assert(computed==101);
    for(uint16_t w : {101,65535}) {
      widths.get(w,[&](){++computed; return control_math::WidthPrediction{0,0};});
    }
    assert(computed==103);
  }
  std::printf("Current/tau models: %u calls match frozen expressions; exact-input invalidation PASS\n",count);
}
int main() {
  modelInputs();
  unsigned hardware=0, logical=0, unique=0;
"""
for e in inputs["records"]:
    cpp += "  { Input r; r.physical_side=" + str(e["physical_side"]) + ";\n"
    for field, value in zip(fields, e["f32_bits"]):
        cpp += "    r." + field + "=scalar(" + str(value) + "U);\n"
    cpp += (
        "    const auto result=compare(r); assert(result.valid);"
        " assert(result.ff==" + str(e["ff_width_ms"]) + " && result.selected==" +
        str(e["selected_width_ms"]) + "); ++hardware; logical+=result.evaluations;"
        " unique+=result.computed;\n  }\n"
    )
cpp += r"""
  assert(hardware==208 && unique<logical);
  std::printf("Frozen legacy cache decisions: %u exact matches; logical evaluations=%u, computed=%u (saved=%u) PASS\n",
      hardware,logical,unique,logical-unique);
  // Reuse the actual physics expressions for random input sets. Includes both
  // signs, opposite residual current, integral saturation and no-output states.
  for(unsigned i=0;i<20000;++i) {
    Reference m;
    Input r;
    r.physical_side=(i&1) ? 1 : -1;
    r.i0_mA=350*noise(); r.free_peak_deg=8+8*noise();
    r.target_peak_deg=8+8*noise();
    r.base_gain=0.2f+0.15f*noise();
    r.correction_c=4*noise(); r.correction_gain=r.base_gain*(1+noise());
    r.integral_mA_s=40*noise();
    r.signed_target_current_mA=r.physical_side*m.predictCurrentGoalMa(300,4+0.4f*noise());
    r.tau_s=m.predictRiseTauS(300); m.r=r;
    r.target_energy_j=m.energyControlPotentialJ(r.target_peak_deg);
    r.passive_energy_j=m.energyControlPotentialJ(r.free_peak_deg);
    r.q_available_mA_s=fabsf(r.signed_target_current_mA*0.1f+
        (r.i0_mA-r.signed_target_current_mA)*r.tau_s*(1-expf(-0.1f/r.tau_s)));
    compare(r);
    if(i<100) {
      r.target_energy_j=NAN;compare(r);
      r.tau_s=0;compare(r);
      r.free_peak_deg=1000;compare(r);
    }
  }
  std::puts("20,000 additional decisions + 300 invalid/boundary cases match frozen search/targets/zero-output PASS");
}
"""
with tempfile.TemporaryDirectory(prefix="control-cache-test-") as temp:
    path = Path(temp)
    (path / "test.cpp").write_text(cpp)
    binary = path / "test"
    subprocess.run(["g++", "-std=c++17", "-Os", "-ffp-contract=off", "-Wall", "-Wextra", "-Werror",
                    "-I" + str(ROOT / "tools/host_v46o"), "-I" + str(ROOT / "src"),
                    str(path / "test.cpp"), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
