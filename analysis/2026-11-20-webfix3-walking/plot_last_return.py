"""Plot the two webfix3 trials from the output directories of convert_rwlog_to_csv.py."""
import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager
from matplotlib.lines import Line2D
from matplotlib.patches import Patch


def rows(directory, filename):
    with (directory / filename).open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def valid(value):
    return value in ("1", "True", "true")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run1", type=Path)
    parser.add_argument("run2", type=Path)
    parser.add_argument("--out", type=Path, default=Path(__file__).with_name("last-return.svg"))
    args = parser.parse_args()
    plt.rcParams.update({"font.family": ["Meiryo", "Noto Sans CJK JP", "sans-serif"],
                         "font.size": 10, "axes.unicode_minus": False,
                         "svg.hashsalt": "webfix3-final-return"})
    font = Path("C:/Windows/Fonts/meiryo.ttc")
    if font.is_file():
        font_manager.fontManager.addfont(str(font))
        plt.rcParams["font.family"] = "Meiryo"
    body, left, right, pulse, current = "#253c58", "#087a68", "#b04f72", "#efa93f", "#60559b"
    fig, axes = plt.subplots(3, 2, figsize=(11.3, 8.0),
                             gridspec_kw={"height_ratios": [1.05, 1.4, .75]})
    fig.subplots_adjust(left=.092, right=.98, bottom=.118, top=.825, hspace=.53, wspace=.18)
    fig.suptitle("歩行が止まる直前の胴体・足角・ホイール電流", fontsize=18, fontweight="bold", y=.99)
    fig.legend(handles=[Line2D([], [], color=body, label="胴体の左右角（MEKF）"),
                        Line2D([], [], color=left, marker=".", label="左足のカメラ角"),
                        Line2D([], [], color=right, marker=".", label="右足のカメラ角"),
                        Patch(facecolor=pulse, alpha=.36, label="最後の通電指令")],
               ncol=4, loc="upper center", bbox_to_anchor=(.54, .943), frameon=False)
    for col, directory in enumerate((args.run1, args.run2)):
        samples = [r for r in rows(directory, "timeseries.csv") if r["state_id"] == "3"]
        feet = [r for r in rows(directory, "foot_angles.csv")
                if r["state_id"] == "3" and valid(r["timestamp_valid"]) and int(r["measurement_time_us"]) >= 0]
        peak = int(rows(directory, "energy_control_autonomous_peak_events.csv")[-1]["peak_time_ms"]) / 1000
        final = [r for r in rows(directory, "walking_energy.csv") if int(r["output_start_ms"] or 0) > 0][-1]
        start, end = (int(final[key]) / 1000 for key in ("output_start_ms", "output_end_ms"))
        t = np.array([int(r["t_test_ms"]) / 1000 for r in samples])
        angle = np.array([float(r["pitch_mekf_detector_relative_deg"]) for r in samples])
        ft = np.array([int(r["measurement_time_us"]) / 1e6 for r in feet])
        overview, angles, currents = axes[:, col]
        for ax in (overview, angles, currents):
            ax.spines[["top", "right"]].set_visible(False)
            ax.spines[["bottom", "left"]].set_color("#aab5bd")
            ax.grid(axis="y", color="#dfe5e8", lw=.7)
            ax.set_axisbelow(True)
            ax.axhline(0, color="#a6afb6", lw=.9, zorder=0)
        overview.set_title(f"{col + 1}回目　最後の正側ピーク {peak:.3f} 秒",
                           fontsize=12, fontweight="bold", loc="left", pad=8)
        overview.plot(t, angle, color=body, lw=1.25)
        overview.axvspan(peak - .22, peak + 1.07, color="#e9ebef", zorder=0)
        overview.set(xlim=(0, 30), ylim=(-7.3, 11.5), xticks=range(0, 31, 5),
                     yticks=[-5, 0, 5, 10], xlabel="測定開始からの時間（秒）")
        angles.plot(t - peak, angle, color=body, lw=2)
        for side, color in (("left", left), ("right", right)):
            degrees = [float(r[side + "_deg"]) if all(valid(r[k]) for k in
                       ("frame_valid", "zero_ready", side + "_valid")) else np.nan for r in feet]
            angles.plot(ft - peak, degrees, ".-", color=color, lw=.9, ms=4)
        # Both first return arcs reverse in this window. Later smaller oscillations are separate.
        window = np.flatnonzero((t > peak + .2) & (t < peak + .65))
        trough = window[np.argmin(angle[window])]
        angles.scatter(t[trough] - peak, angle[trough], s=37, color="white", edgecolor=body, zorder=5)
        angles.annotate(f"折り返し 約 {angle[trough]:+.2f}°", (t[trough] - peak, angle[trough]),
                        (.57, 5.7), color=body, fontsize=9,
                        arrowprops={"arrowstyle": "-", "color": body, "lw": .8})
        angles.set_title("最後の正側ピーク付近を拡大", fontsize=10, loc="left", pad=2)
        angles.set(ylim=(-4.2, 12), yticks=[-4, 0, 4, 8, 12])
        epoch = int(feet[0]["frame_us"]) - int(feet[0]["measurement_time_us"])
        stamps, measured = [], []
        previous = None
        for r in samples:
            stamp = int(r["roller_current_sample_time_us"] or 0)
            if stamp and stamp != previous and valid(r["roller_current_valid"]):
                stamps.append(((stamp - epoch) & 0xffffffff) / 1e6 - peak)
                measured.append(float(r["roller_actual_current_mA"]))
                previous = stamp
        currents.plot(stamps, measured, ".-", ms=2.7, lw=1.2, color=current)
        currents.set(ylim=(-300, 500), yticks=[-250, 0, 250, 450],
                     xlabel="最後の正側ピークからの時間（秒）")
        for ax in (angles, currents):
            ax.axvspan(start - peak, end - peak, color=pulse, alpha=.36, zorder=1)
            ax.set(xlim=(-.22, 1.07), xticks=np.arange(-.2, 1.01, .2))
        angles.tick_params(labelbottom=False)
        if col == 0:
            overview.set_ylabel("胴体の左右角（°）")
            angles.set_ylabel("角度（°）")
            currents.set_ylabel("実測電流（mA）")
    fig.text(.09, .025, "上段の灰色部分を中・下段に拡大。足角は撮影時刻に置いた有効な観測点で、両足とも前振れが正。",
             fontsize=9, color="#4b5960")
    meta = {"Date": None} if args.out.suffix.lower() == ".svg" else {}
    fig.savefig(args.out, dpi=105, metadata=meta)
    plt.close(fig)
    if args.out.suffix.lower() == ".svg":
        lines = args.out.read_bytes().splitlines()
        args.out.write_bytes(b"\n".join(line.rstrip() for line in lines) + b"\n")


if __name__ == "__main__":
    main()
