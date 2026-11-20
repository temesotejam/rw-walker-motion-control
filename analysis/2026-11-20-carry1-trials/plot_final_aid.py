"""Plot the last actual return aid and the carry1 forecasts available around it."""
import argparse
import csv
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from score_observed_transfers import capture_us, gyro_samples, measurement_epoch


def rows(directory, name):
    with (directory / name).open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def draw(directory, axes, index):
    metadata = json.loads((directory / "metadata.json").read_text(encoding="utf-8"))
    epoch = measurement_epoch(metadata)
    if epoch is None:
        raise ValueError("The IMU and camera do not share a verified measurement clock")
    pulses = [r for r in rows(directory, "walking_energy.csv") if r["rescue"] == r["executed"] == "True"]
    pulse = pulses[-1]
    start, end = (int(pulse[key]) for key in ("output_start_ms", "output_end_ms"))
    peak = pulse["peak_ms"]
    plans = [r for r in rows(directory, "support_transfer_shadow.csv")
             if r["peak_ms"] == peak and r["phase"] == "2" and r["coast_margin_j"]]
    t, _, angle = gyro_samples(metadata, rows(directory, "timeseries.csv"), epoch)
    motion, energy = axes
    motion.plot([x-start for x in t], angle, color="#29415b", label="胴体の左右角")
    feet = [r for r in rows(directory, "foot_angles.csv") if r["state_id"] == "3"
            and all(r[k] == "True" for k in ("timestamp_valid", "frame_valid", "zero_ready", "left_valid"))
            and start-200 <= int(r["measurement_time_us"])/1000 <= start+450]
    motion.plot([int(r["measurement_time_us"])/1000-start for r in feet],
                [float(r["left_deg"]) for r in feet], ".-", color="#297964", lw=1, label="左足の前振れ")
    motion.axhline(float(plans[0]["terminal_deg"]), color="#70777c", ls="--", lw=1.2, label="受け側の終端角")
    motion.set_title(f"{index}回目　実際の救済通電 {start/1000:.3f}–{end/1000:.3f} 秒", loc="left", fontsize=11)
    motion.set_ylim(-3,13)
    for row in plans:
        capture = capture_us(row, epoch)/1000-start
        ready = capture + int(row["ready_after_us"])/1000
        if ready < -170 or capture > 400:
            continue
        margin = float(row["coast_margin_j"])*1000
        color = "#187765" if margin > 0 else "#a63e50"
        energy.plot([capture,ready], [margin,margin], color=color, lw=1.2)
        energy.plot(ready, margin, marker="o", markersize=4, color=color)
    for ax in axes:
        ax.axvspan(0,end-start,color="#e7ad52",alpha=.35)
        ax.axhline(0,color="#8998a0",lw=.7)
        ax.set_xlim(-170,400)
        ax.grid(axis="y", alpha=.18)
        ax.spines[["right","top"]].set_visible(False)
    energy.set_ylim(-4,4)
    energy.set_xlabel("救済通電の開始からの時間（ms）")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trials", type=Path, nargs=2, help="Converted run 1 and 2 directories")
    parser.add_argument("--out", type=Path, default=Path(__file__).with_name("final-aid.svg"))
    args = parser.parse_args()
    plt.rcParams.update({"font.family":"Meiryo", "svg.fonttype":"none",
                         "svg.hashsalt":"carry1-final-aid", "axes.unicode_minus":False})
    fig, axes = plt.subplots(2,2,figsize=(11,5.6),sharex="col",sharey="row")
    for i, trial in enumerate(args.trials):
        draw(trial,axes[:, i],i+1)
    axes[0,0].set_ylabel("角度（°）")
    axes[1,0].set_ylabel("惰性で進む場合の余裕（mJ）")
    fig.legend(*axes[0,0].get_legend_handles_labels(),loc="upper center",ncol=3,frameon=False)
    fig.subplots_adjust(left=.09,right=.98,top=.83,bottom=.18,hspace=.22,wspace=.12)
    fig.text(.09,.038,"橙は実際の通電期間。下段の横線は観測の取得から判断完了まで、点は判断が利用可能になった時刻。")
    fig.savefig(args.out,metadata={"Date":None})
    plt.close(fig)
    if args.out.suffix.lower() == ".svg":
        # Matplotlib places a trailing space on each path vertex line.
        lines = args.out.read_text(encoding="utf-8").splitlines()
        with args.out.open("w", encoding="utf-8", newline="\n") as stream:
            stream.write("\n".join(line.rstrip() for line in lines) + "\n")


if __name__ == "__main__":
    main()
