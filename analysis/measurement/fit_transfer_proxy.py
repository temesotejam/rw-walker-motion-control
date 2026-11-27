"""Predict signed lateral rate around foot-edge events identified in video.

Labels are apparent lower-foot switches. Supply independently selected image
intervals. The event clock is given to every model, including the baselines.
"""
import argparse
import csv
import json
from bisect import bisect_left, bisect_right
from pathlib import Path

import numpy as np

from measure_transfer_speeds import (CausalFootFeatures, command_archive, command_overlap,
                                     gyro_samples, input_evidence, led_edges, video_clock)

PRE_MS = 50.0
POST_MS = 1000 / 30
MODELS = {"side_mean": (), "entry_speed": ("entry_dps",),
          "entry_and_feet": ("entry_dps", "receiving_deg", "other_deg")}


def read_csv(path):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


class Trial:
    def __init__(self, decoded, events, anchors):
        header = json.loads((decoded / "header.json").read_text(encoding="utf8"))
        meta = json.loads((decoded / "metadata.json").read_text(encoding="utf8"))
        self.run = int(header["run_start_us"])
        self.events = read_csv(events)
        samples = read_csv(decoded / "timeseries.csv")
        lo, hi, self.clocks = video_clock(self.run, led_edges(header, meta, samples), read_csv(anchors))
        previous_hi = float("-inf")
        for e in self.events:
            if int(e["run_start_us"]) != self.run or e["apparent_lower_side_after"] not in ("L","R"):
                raise ValueError("Camera event has an unexpected run or side")
            if not lo <= float(e["video_lo_ms"]) < float(e["video_hi_ms"]) <= hi:
                raise ValueError("Event lies outside LED synchronization")
            if float(e["video_lo_ms"]) <= previous_hi:
                raise ValueError("Independent event intervals must be distinct and chronological")
            previous_hi = float(e["video_hi_ms"])
        self.camera = CausalFootFeatures(meta)
        origin = self.camera.epoch_us
        self.ts, self.rate, _ = gyro_samples(meta, samples, origin)
        ticks = {}
        for row in samples:
            if row["state_id"] == "3" and row["body_rate_sample_time_us"]:
                stamp = (int(row["body_rate_sample_time_us"]) - origin) & 0xffffffff
                # Integer control tick has 1 ms resolution. Use its upper edge.
                ticks.setdefault(stamp, float(row["t_test_ms"]) + 1)
        self.ready = [max(t, ticks[round(t*1000)]) for t in self.ts]
        if any(b < a for a,b in zip(self.ready, self.ready[1:])):
            raise ValueError("IMU receipt clock moved backwards")
        self.active, self.currents, applied = input_evidence(samples, origin)
        self.archive = command_archive(meta, applied)

    def rows(self, clock=None, phase=.5):
        result = []
        for e in self.events:
            side = e["apparent_lower_side_after"]
            direction = 1 if side == "R" else -1
            lo, hi = float(e["video_lo_ms"]), float(e["video_hi_ms"])
            pts = lo + phase*(hi-lo)
            if clock is None:
                possible = [a*pts+b for a,b in self.clocks]
                event = (min(possible)+max(possible))/2
            else:
                event = clock[0]*pts+clock[1]
            cutoff, after = event-PRE_MS, event+POST_MS
            pre = bisect_right(self.ready,cutoff)-1
            post = bisect_left(self.ts,after)
            if pre < 0 or post >= len(self.ts) or cutoff-self.ts[pre] > 16 or self.ts[post]-after > 10:
                continue
            foot = self.camera.foot_at_us(self.camera.epoch_us+cutoff*1000)
            left, right = foot["left_deg"], foot["right_deg"]
            if left is None or right is None:
                continue
            overlap = command_overlap(self.archive, self.ts[pre], self.ts[post])
            if any(self.ts[pre] <= t <= self.ts[post] for t in self.active):
                overlap = "overlap"
            current = [v for t,v in self.currents if self.ts[pre] <= t <= self.ts[post]]
            result.append(dict(run=self.run, frame=int(e["frame_lo"]), side=side, event_ms=event,
                               entry_cutoff_ms=cutoff, entry_imu_ms=self.ts[pre], entry_receipt_ms=self.ready[pre],
                               foot_age_ms=foot["foot_age_ms"], receiving_deg=right if side=="R" else left,
                               other_deg=left if side=="R" else right, entry_dps=direction*self.rate[pre],
                               post_ms=self.ts[post], post_dps=direction*self.rate[post],
                               command_overlap=overlap, max_abs_current_ma=max(current, default=None)))
        return result


def fit(rows, fields):
    # Shrink the speed slope toward persistence and foot slopes toward zero;
    # each side has its own unpenalized intercept. Fix penalty before validation.
    sides = np.asarray([r["side"]=="R" for r in rows])
    if len(set(sides)) != 2:
        raise ValueError("Both sides are needed in the training partition")
    x = np.asarray([[r[f] for f in fields] for r in rows], dtype=float)
    y = np.asarray([r["post_dps"] for r in rows])
    prior = np.asarray([float(f=="entry_dps") for f in fields])
    residual = y-x@prior
    xc, yc = x.copy(), residual.copy()
    for mask in (sides, ~sides):
        xc[mask] -= x[mask].mean(axis=0)
        yc[mask] -= residual[mask].mean()
    slopes = prior.copy()
    if fields:
        scale = np.sqrt((xc**2).mean(axis=0))
        keep = scale > 1e-8
        a = xc[:,keep]/scale[keep]
        slopes[keep] += np.linalg.solve(a.T@a+np.eye(keep.sum()),a.T@yc)/scale[keep]
    offsets = {s: float((y-x@slopes)[sides==(s=="R")].mean()) for s in ("L","R")}
    return dict(offset_dps=offsets, coefficients=dict(zip(fields,slopes.tolist())))


def predict(model, row):
    return model["offset_dps"][row["side"]] + sum(c*row[f] for f,c in model["coefficients"].items())


def score(rows, models):
    scores = {}
    for name in ("persistence", *models):
        def error(r):
            p = r["entry_dps"] if name=="persistence" else predict(models[name], r)
            return abs(p-r["post_dps"])
        groups = {"all":rows, **{s:[r for r in rows if r["side"]==s] for s in ("L","R")}}
        scores[name] = {
            side: dict(n=len(group), mae_dps=float(np.mean([error(r) for r in group])))
            for side, group in groups.items() if group
        }
    return scores


def sensitivity(trial, central, models):
    central_frames = {r["frame"] for r in central}
    scenarios = [[r for r in trial.rows(clock,phase) if r["frame"] in central_frames]
                 for clock in trial.clocks for phase in (0.,.5,1.)]
    common = central_frames.intersection(*({r["frame"] for r in trial_rows} for trial_rows in scenarios))
    if not common:
        return dict(n=0, scenarios=len(scenarios))
    results = [score([r for r in rows if r["frame"] in common],models) for rows in scenarios]
    return dict(n=len(common),scenarios=len(scenarios),frames=sorted(common),
                mae_range_dps={k:[min(s[k]["all"]["mae_dps"] for s in results),
                                  max(s[k]["all"]["mae_dps"] for s in results)] for k in results[0]},
                target_range_dps={f:[min(r["post_dps"] for rows in scenarios for r in rows if r["frame"]==f),
                                     max(r["post_dps"] for rows in scenarios for r in rows if r["frame"]==f)]
                                  for f in sorted(common)})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("decoded",type=Path)
    parser.add_argument("events",type=Path,help="run_start_us, apparent_lower_side_after, frame_lo, video_lo_ms, video_hi_ms")
    parser.add_argument("anchors",type=Path)
    parser.add_argument("--split-s",type=float,required=True)
    parser.add_argument("--external",nargs=3,type=Path,metavar=("DECODED","EVENTS","ANCHORS"))
    parser.add_argument("--out",type=Path,required=True)
    args = parser.parse_args()
    primary = Trial(args.decoded,args.events,args.anchors)
    rows = primary.rows()
    train = [r for r in rows if r["post_ms"] < args.split_s*1000]
    held = [r for r in rows if r["entry_cutoff_ms"] >= args.split_s*1000]
    if not held:
        parser.error("No holdout events")
    models = {name:fit(train,fields) for name,fields in MODELS.items()}
    partitions = [("train",primary,train),("heldout",primary,held)]
    if args.external:
        other = Trial(*args.external)
        if other.run == primary.run:
            parser.error("External validation requires another run")
        partitions.append(("external",other,other.rows()))
    ranges = {}
    for side in ("L", "R"):
        ranges[side] = {}
        for field in MODELS["entry_and_feet"]:
            values = [r[field] for r in train if r["side"] == side]
            ranges[side][field] = [min(values), max(values)]
    report = dict(proxy="apparent lower side after visible foot-edge band reversal",
                  lead_ms=PRE_MS, post_offset_ms=POST_MS, split_s=args.split_s, models=models,
                  ridge_penalty=1., calibration_features=ranges, partitions={})
    output = []
    for name,trial,group in partitions:
        summary=dict(run=trial.run, labelled_in_run=len(trial.events), score=score(group,models))
        if group:
            observed = [row["post_dps"] for row in group]
            summary["observed_dps_range"] = [min(observed), max(observed)]
            summary["command_overlap_count"] = sum(row["command_overlap"] == "overlap" for row in group)
        if name == "external":
            accepted = {row["frame"] for row in group}
            summary["excluded_frames"] = [int(e["frame_lo"]) for e in trial.events if int(e["frame_lo"]) not in accepted]
        if name!="train":
            summary["clock_and_frame_sensitivity"] = sensitivity(trial,group,models)
        report["partitions"][name]=summary
        for row in group:
            output.append(dict(partition=name,**row,**{k:predict(model,row) for k,model in models.items()}))
    args.out.mkdir(parents=True,exist_ok=True)
    (args.out/"fit.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf8")
    with (args.out/"predictions.csv").open("w",newline="",encoding="utf8") as stream:
        writer=csv.DictWriter(stream,fieldnames=output[0])
        writer.writeheader()
        writer.writerows(output)


if __name__=="__main__":
    main()
