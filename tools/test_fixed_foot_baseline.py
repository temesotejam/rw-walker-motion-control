#!/usr/bin/env python3
"""Refit the measured fixture and check the production C++ predictor.

Standard library only. Leave out a whole run, never random events from the
same run. This evaluates one-step predictions using recorded Q, not the
closed-loop behavior that a different command would have caused.
"""
import json
import math
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
DATA = json.loads((ROOT / "tools/fixtures/fixed_foot_20260929.json").read_text())
FIELDS = DATA["fields"]
RUNS = [[dict(zip(FIELDS, row)) for row in rows] for rows in DATA["rows"]]
GAIN = {1: 0.29032, -1: 0.25455}


def predict(row, parameters):
    intercept, slope = parameters
    return max(0.0, intercept + slope * (row["rate_dps"] - 65.0)) + GAIN[row["side"]] * row["q_mA_s"]


def fit(rows):
    """Least squares in each linear region of the zero-floored model."""
    intercept, slope = 7.0, 0.18
    previous_active = None
    for _ in range(100):
        active = tuple(i for i, row in enumerate(rows)
                       if intercept + slope * (row["rate_dps"] - 65.0) > 0)
        if active == previous_active:
            assert math.isfinite(intercept) and slope > 0
            return intercept, slope
        previous_active = active
        points = [(rows[i]["rate_dps"] - 65.0,
                   rows[i]["next_peak_deg"] - GAIN[rows[i]["side"]] * rows[i]["q_mA_s"])
                  for i in active]
        assert len(points) >= 2
        x_mean = sum(x for x, _ in points) / len(points)
        y_mean = sum(y for _, y in points) / len(points)
        slope = sum((x - x_mean) * (y - y_mean) for x, y in points) / sum((x - x_mean) ** 2 for x, _ in points)
        intercept = y_mean - slope * x_mean
    raise AssertionError("zero-floor fit did not converge")


def rmse(errors):
    return math.sqrt(sum(e * e for e in errors) / len(errors))


def main():
    rows = [row for run in RUNS for row in run]
    assert DATA["schema"] == 1 and len(RUNS) == 3 and len(rows) == 204
    assert [source["target_deg"] for source in DATA["sources"]] == [8, 10, 12]
    assert sum(row["side"] == 1 for row in rows) == 103
    for row in rows:
        assert all(math.isfinite(value) for value in row.values())
        assert row["side"] in (-1, 1) and row["rate_dps"] >= 0 and row["q_mA_s"] >= 0
    parameters = {side: fit([row for row in rows if row["side"] == side]) for side in (-1, 1)}

    # Evaluate the actual float C++ implementation and the production gain and
    # residual-disable configuration, rather than a second hardcoded formula.
    harness = r'''
#include <cassert>
#include <iomanip>
#include <iostream>
#include "rate_baseline_correction.h"
#include "config.h"
int main() {
  static_assert(!Config::ENERGY_CONTROL_AUTONOMOUS_PREVIOUS_PEAK_CONTROL_ENABLED,
                "The retired 8-degree residual must not be stacked on the refit");
  assert(rate_baseline::evaluate(NAN, 1).reason == rate_baseline::INVALID_INPUT);
  assert(rate_baseline::evaluate(-1, 1).reason == rate_baseline::INVALID_INPUT);
  assert(rate_baseline::evaluate(65, 0).reason == rate_baseline::INVALID_INPUT);
  int side; float rate, q;
  while (std::cin >> side >> rate >> q) {
    const auto baseline = rate_baseline::evaluate(rate, side);
    const float gain = side > 0 ? Config::Q1_SHADOW_GAIN_PHYSICAL_PLUS_DEG_PER_MAS
                               : Config::Q1_SHADOW_GAIN_PHYSICAL_MINUS_DEG_PER_MAS;
    std::cout << std::setprecision(10) << baseline.adjusted_deg + gain*q << '\n';
  }
}
'''
    probes = rows + [dict(side=side, rate_dps=rate, q_mA_s=0.0)
                     for side in (-1, 1) for rate in (0.0, 65.0, 80.0, 120.0)]
    with tempfile.TemporaryDirectory(prefix="fixed-foot-regression-") as directory:
        source = Path(directory) / "predict.cpp"
        executable = Path(directory) / "predict"
        source.write_text(harness)
        subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
                        "-I" + str(ROOT / "tools/host_v46o"), "-I" + str(ROOT / "src"),
                        str(source), "-o", str(executable)], check=True)
        input_text = "".join(f'{r["side"]} {r["rate_dps"]} {r["q_mA_s"]}\n' for r in probes)
        result = subprocess.run([str(executable)], input=input_text, text=True,
                                capture_output=True, check=True)
    actual = [float(line) for line in result.stdout.splitlines()]
    assert len(actual) == len(probes)
    assert max(abs(value - predict(row, parameters[row["side"]]))
               for value, row in zip(actual, probes)) < 1e-5

    old_errors = [row["old_prediction_deg"] - row["next_peak_deg"] for row in rows]
    fit_errors = [prediction - row["next_peak_deg"] for prediction, row in zip(actual, rows)]
    held_out_errors = []
    for held_out, run in enumerate(RUNS):
        training = [row for index, other in enumerate(RUNS) if index != held_out for row in other]
        fold = {side: fit([row for row in training if row["side"] == side]) for side in (-1, 1)}
        errors = [predict(row, fold[row["side"]]) - row["next_peak_deg"] for row in run]
        held_out_errors.extend(errors)
        old_run = [row["old_prediction_deg"] - row["next_peak_deg"] for row in run]
        print(f'{DATA["sources"][held_out]["target_deg"]}deg: n={len(run)}, '
              f'old RMSE={rmse(old_run):.6f}, held-out RMSE={rmse(errors):.6f}')
    assert abs(rmse(old_errors) - 1.412430) < 1e-5
    assert abs(rmse(fit_errors) - 0.318973) < 1e-5
    assert abs(rmse(held_out_errors) - 0.497029) < 1e-5
    print(f"Fixed-foot 204-event predictor: old RMSE={rmse(old_errors):.6f}, "
          f"fit RMSE={rmse(fit_errors):.6f}, leave-one-run-out RMSE={rmse(held_out_errors):.6f} deg")
    for side in (-1, 1):
        subset = [r for r in rows if r["side"] == side]
        print(f"side={side:+d}: intercept={parameters[side][0]:.12f}, "
              f"slope={parameters[side][1]:.12f}, n={len(subset)}, "
              f'rate range={min(r["rate_dps"] for r in subset):.4f}..'
              f'{max(r["rate_dps"] for r in subset):.4f} dps')


if __name__ == "__main__":
    main()
