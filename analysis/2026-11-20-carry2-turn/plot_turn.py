"""Plot synchronized onboard foot measurements and relative gyro heading for run 50784949."""
import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def rows(folder, name):
    with (folder / name).open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("converted", type=Path)
    parser.add_argument("--out", type=Path, default=Path(__file__).with_name("foot-heading.svg"))
    args = parser.parse_args()
    keys = ("timestamp_valid", "frame_valid", "zero_ready", "left_valid", "right_valid")
    feet = [r for r in rows(args.converted, "foot_angles.csv") if r["state_id"] == "3"
            and all(r[k] == "True" for k in keys) and 0 <= int(r["measurement_time_us"]) <= 18000000]
    body = [r for r in rows(args.converted, "timeseries.csv") if r["state_id"] == "3"
            and r["gyro_heading_valid"] == "1" and 0 <= int(r["t_test_ms"]) <= 18000]
    t = [int(r["measurement_time_us"]) / 1e6 for r in feet]
    plt.rcParams.update({"font.family": "Meiryo", "svg.fonttype": "none",
                         "svg.hashsalt": "carry2-turn-50784949", "axes.unicode_minus": False})
    fig, (foot, yaw) = plt.subplots(2, 1, figsize=(9.6, 4.9), sharex=True, layout="constrained")
    for name, label, marker, color in (("left_deg", "機体左", "o", "#156b63"),
                                        ("right_deg", "機体右", "^", "#b05725")):
        foot.scatter(t, [float(r[name]) for r in feet], s=6, marker=marker, color=color, label=label)
    foot.set_ylabel("前振れ角（°）")
    foot.legend(loc="upper left", ncols=2, frameon=False)
    foot.set_title("機体内カメラの左右足角とジャイロの向き　測定開始から18秒", loc="left")
    yaw.plot([int(r["t_test_ms"]) / 1000 for r in body], [float(r["gyro_heading_deg"]) for r in body],
             color="#344b69", lw=1.3)
    yaw.set_ylabel("相対ジャイロ方位（°）")
    yaw.set_xlabel("測定開始からの時間（秒）")
    for ax in (foot, yaw):
        ax.set_xlim(0, 18)
        ax.set_xticks(range(0, 19, 2))
        ax.grid(axis="y", alpha=.18)
        ax.spines[["right", "top"]].set_visible(False)
    fig.savefig(args.out, metadata={"Date": None})
    plt.close(fig)
    if args.out.suffix.lower() == ".svg":
        lines = args.out.read_text(encoding="utf-8").splitlines()
        with args.out.open("w", encoding="utf-8", newline="\n") as stream:
            stream.write("\n".join(line.rstrip() for line in lines) + "\n")


if __name__ == "__main__":
    main()
