"""Plot the two measured runs from their standard RWLOG conversion folders."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def csv_rows(folder, name):
    with (folder / (name + ".csv")).open(encoding="utf-8-sig", newline="") as source:
        return list(csv.DictReader(source))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("converted_root", type=Path, help="Parent of folders named 33125639 and 216664125")
    parser.add_argument("output", type=Path, help="PNG output")
    args = parser.parse_args()
    plt.rcParams.update({"font.family": ["Yu Gothic", "DejaVu Sans"], "font.size": 10})
    fig, axes = plt.subplots(2, 2, figsize=(11, 5.6), sharex=True, sharey="row", layout="constrained")
    for col, (run, profile) in enumerate((("33125639", "turn3"), ("216664125", "v14_small_step"))):
        folder = args.converted_root / run
        metadata = json.loads((folder / "metadata.json").read_text(encoding="utf-8"))
        if metadata.get("walking_profile") != profile:
            raise ValueError(f"Unexpected walking profile for {run}")
        body, feet = axes[:, col]
        imu = [r for r in csv_rows(folder, "timeseries") if r["state_id"] == "3" and int(r["t_test_ms"]) <= 8000]
        at = [int(r["t_test_ms"]) * .001 for r in imu]
        roll = [float(r["pitch_mekf_detector_relative_deg"]) for r in imu]
        body.axhline(0, lw=1, color="#64748b")
        body.axhline(4.5, lw=1, ls="--", color="#697994", label="起動時の右目標 +4.5°")
        body.plot(at, roll, lw=1.5, color="#183f69", label="胴体の左右角")
        peak = metadata["energy_control_autonomous_peak_events"][-1]
        t0 = peak["peak_time_ms"] * .001
        t, angle = min(((t, ang) for t, ang in zip(at, roll) if t0 <= t <= t0 + .65), key=lambda x: x[1])
        body.plot(t, angle, "o", color="#ad451d", ms=6)
        body.annotate(f"中心手前 +{angle:.2f}°", (t, angle), (t + .10, -2.7), color="#96380f",
                      arrowprops={"arrowstyle": "-", "color": "#ad451d", "lw": 1})
        for ax in (body, feet):
            ax.axvline(t, lw=0.8, color="#ad451d", alpha=.6, ls=":")
            ax.set_xlim(0, 8)
            ax.spines[["top", "right"]].set_visible(False)
            ax.grid(axis="y", color="#dfe4ea", lw=.6)
        body.set_title(("1走目　turn3" if col == 0 else "2走目　v14式小歩幅") + f"　({run})")
        foot = [r for r in csv_rows(folder, "foot_periodic") if int(r["slot_us"]) <= 8_000_000]
        time = [int(r["slot_us"]) * 1e-6 for r in foot]
        for field, label, color, style in (("left_observed_deg", "左足", "#a44826", "-"),
                                            ("right_observed_deg", "右足", "#007d71", "--")):
            feet.plot(time, [float(r[field]) if r[field] else float("nan") for r in foot],
                      label=label, color=color, ls=style, lw=1.4)
        feet.set_xlabel("測定開始からの時間 (s)")
    axes[0, 0].set_ylabel("胴体の左右角 (°)")
    axes[1, 0].set_ylabel("足の前振れ角 (°)")
    axes[0, 1].legend(loc="upper right", fontsize=8, ncol=2, frameon=False)
    axes[1, 0].legend(loc="upper right", ncol=2, frameon=False)
    fig.suptitle("旧版2走の実測　最後の右ピークから左へ戻る区間", fontsize=14)
    fig.savefig(args.output, dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
