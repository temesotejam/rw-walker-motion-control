"""Compare recorded video with a fixed-view CAD reconstruction from a decoded RWLOG."""
import argparse
import csv
import json
import struct
import sys
from pathlib import Path

import cv2
import numpy as np


HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "support"))
from score_observed_transfers import measurement_epoch
from analyze_energy_neutral_v14 import SOURCE, load_feet, reconstruct_sole_arcs

RIGHT = (38, 104, 195)
LEFT = (187, 83, 47)


def load_meshes(source, hip):
    glb = source.read_bytes()
    n = struct.unpack_from("<I", glb, 12)[0]
    doc = json.loads(glb[20:20+n])
    blob = glb[28+n:]
    def array(i, dimensions):
        acc = doc["accessors"][i]
        view = doc["bufferViews"][acc["bufferView"]]
        dtype = {5126:"<f4",5125:"<u4",5123:"<u2",5121:"u1"}[acc["componentType"]]
        assert not view.get("byteStride") or view["byteStride"] == np.dtype(dtype).itemsize*dimensions
        return np.frombuffer(blob, dtype=dtype, offset=view.get("byteOffset",0)+acc.get("byteOffset",0),
                             count=acc["count"]*dimensions).reshape(-1,dimensions)
    meshes = []
    for node in doc["nodes"]:
        if "mesh" not in node:
            continue
        idx = int(node["name"].split("_")[-1])
        prim = doc["meshes"][node["mesh"]]["primitives"][0]
        points = array(prim["attributes"]["POSITION"],3).astype(float)-hip
        tri = array(prim["indices"],1).reshape(-1,3)
        meshes.append((idx,points,tri))
    return meshes


def rotate(points, pitch, roll):
    a,b = np.deg2rad([pitch,roll])
    x,y,z = points.T
    yp = np.cos(a)*y-np.sin(a)*z
    zp = np.sin(a)*y+np.cos(a)*z
    return np.stack([np.cos(b)*x+np.sin(b)*zp,yp,-np.sin(b)*x+np.cos(b)*zp],axis=1)


def project(p):
    # A fixed elevated front view and orthographic scale throughout the replay.
    return np.rint(np.c_[246-1.73*(.98*p[:,0]-.20*p[:,1]),
                         327-1.73*(p[:,2]*.986-(.20*p[:,0]+.98*p[:,1])*.165)]).astype(np.int32)


def draw_model(row, meshes):
    panel = np.full((360,492,3),(248,249,246),np.uint8)
    if not row["valid"]:
        return panel
    g=-min(row["right_min_z_mm"],row["left_min_z_mm"])
    for n in range(-100,101,20):
        for p in (np.array([[n,-85.,0],[n,85.,0]]), np.array([[-105.,n,0],[105.,n,0]])):
            a,b=project(p)
            cv2.line(panel,tuple(a),tuple(b),(212,217,211),1,cv2.LINE_AA)
    parts=[]
    for idx,points,tri in meshes:
        side = "right" if idx in (0,5,6,10) else "left" if idx in (1,4,7,12) else None
        color = (RIGHT if side=="right" else LEFT) if side else (104,116,122)
        if idx in (4,6,10,12):
            color=(230,230,213)
        if idx==9: color=(93,111,124)
        p=rotate(points,row["fore_aft_deg"]+(row[side+"_deg"] if side else 0),row["roll_deg"])
        p[:,2]+=g
        # Draw silhouette from front-facing mesh triangles; preserve open regions.
        q=project(p)
        t=q[tri]
        e=t[:,1]-t[:,0]
        f=t[:,2]-t[:,0]
        visible=t[(e[:,0]*f[:,1]-e[:,1]*f[:,0])<0]
        if len(visible):
            parts.append((float((.2*p[:,0]+.98*p[:,1]+.165*p[:,2]).mean()),color,visible))
    for _,color,triangles in sorted(parts,key=lambda p:p[0]):
        mask=np.zeros(panel.shape[:2], np.uint8)
        for tri in triangles:
            cv2.fillConvexPoly(mask,tri,255)
        contours,_=cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        panel[mask!=0]=color
        cv2.drawContours(panel,contours,-1,tuple(round(.67*c) for c in color),1,cv2.LINE_AA)
    return panel


def measured_pose(decoded):
    metadata = json.loads((decoded / "metadata.json").read_text(encoding="utf8"))
    epoch = measurement_epoch(metadata)
    reference = metadata["walking_energy"]["fore_aft_motion_guard"]["start_roll_deg"]
    with (decoded / "timeseries.csv").open(encoding="utf-8-sig", newline="") as stream:
        body = []
        for row in csv.DictReader(stream):
            if row["state_id"] != "3" or not row["body_rate_sample_time_us"] or float(row["log_time_s"]) <= 0:
                continue
            t = ((int(row["body_rate_sample_time_us"])-epoch) & 0xffffffff)/1000
            if t > 600000 or (body and t <= body[-1][0]):
                continue
            body.append((t, float(row["pitch_mekf_detector_relative_deg"]), float(row["roll_mekf_abs_deg"])-reference))
    feet = []
    for frame in sorted(metadata["foot_frames"], key=lambda f: f["frame_us"]):
        if frame["state_id"] != 3 or not frame["timestamp_valid"]:
            continue
        # A RUN frame can be exposed just before the measurement epoch.
        delta = (frame["frame_us"]-epoch + 2**31) % 2**32 - 2**31
        t = delta/1000
        if abs(t) > 600000 or (feet and t <= feet[-1][0]):
            continue
        angles = [frame[side+"_deg"] if frame["frame_valid"] and frame["zero_ready"] and
                  frame[side+"_valid"] and frame[side+"_control_range_ok"] else np.nan for side in ("left","right")]
        feet.append((t, *angles))
    if len(body)<2 or len(feet)<2:
        raise ValueError("This log has insufficient active IMU or time-stamped foot observations")
    return np.asarray(body), np.asarray(feet)


def interpolate(data, t, limit):
    i = data[:,0].searchsorted(t)
    if i < len(data) and data[i,0] == t:
        return data[i,1:]
    if i == 0 or i == len(data):
        return np.full(2, np.nan)
    low,high = data[i-1],data[i]
    if high[0]-low[0] > limit:
        return np.full(2, np.nan)
    return low[1:] + (high[1:]-low[1:]) * ((t-low[0])/(high[0]-low[0]))


def frame_pose(body, feet, soles, t):
    roll,pitch = interpolate(body,t,10)
    left,right = interpolate(feet,t,60)
    row = dict(valid=np.isfinite([roll,pitch,left,right]).all(), roll_deg=roll, fore_aft_deg=pitch,
               left_deg=left, right_deg=right)
    if row["valid"]:
        for side in soles:
            row[side+"_min_z_mm"] = rotate(soles[side],pitch+row[side+"_deg"],roll)[:,2].min()
        row["gap_mm"] = row["left_min_z_mm"]-row["right_min_z_mm"]
    return row


def run(args):
    body, feet = measured_pose(args.decoded)
    points,hip,_ = load_feet(args.cad)
    soles,_ = reconstruct_sole_arcs(points)
    meshes = load_meshes(args.cad,hip)
    if args.clock:
        valid_from, valid_to, vertices = json.loads(args.clock.read_text(encoding="utf8"))
        coeff = np.asarray(vertices, dtype=float)
    else:
        valid_from, valid_to = 0, float("inf")
        coeff = np.array([[1,-1000*args.video_zero_s]])
    cap = cv2.VideoCapture(str(args.video))
    rate = cap.get(cv2.CAP_PROP_FPS)
    if not cap.isOpened() or rate <= 0:
        raise ValueError("Cannot decode the video or its frame rate")
    stride = max(1, round(rate/15))
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    out = cv2.VideoWriter(str(args.output),cv2.VideoWriter_fourcc(*"mp4v"),rate/stride,(1156,480))
    if not out.isOpened():
        raise OSError("MP4 video encoder could not be opened")
    written = 0
    try:
        idx = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            ms = cap.get(cv2.CAP_PROP_POS_MSEC)
            stamps = coeff[:,0]*ms+coeff[:,1]
            t = (stamps.max()+stamps.min())/2
            if ms>valid_to or t>body[-1,0] or t>args.stop_s*1000:
                break
            idx += 1
            if (idx-1)%stride or ms<valid_from or t<args.start_s*1000:
                continue
            pose = frame_pose(body,feet,soles,t)
            panel = np.full((480,1156,3),250,np.uint8)
            scale=min(640/frame.shape[1],360/frame.shape[0])
            view=cv2.resize(frame,None,fx=scale,fy=scale,interpolation=cv2.INTER_AREA)
            y=51+(360-view.shape[0])//2
            x=6+(640-view.shape[1])//2
            panel[y:y+view.shape[0],x:x+view.shape[1]]=view
            panel[51:411,658:1150]=cv2.cvtColor(draw_model(pose,meshes),cv2.COLOR_RGB2BGR)
            def label(x,y,msg,size=.66):
                cv2.putText(panel,msg,(x,y),cv2.FONT_HERSHEY_SIMPLEX,size,(48,54,50),1,cv2.LINE_AA)
            label(12,33,f"Trial video    t = {t/1000:.2f} s")
            label(660,33,"CAD replay from measured body and foot angles")
            label(12,445,"Camera view")
            if pose["valid"]:
                label(660,440,f"Right {pose['right_deg']:+.1f} deg   Left {pose['left_deg']:+.1f} deg   Roll {pose['roll_deg']:+.1f} deg",.58)
                label(660,467,f"Lowest sole height, L - R: {pose['gap_mm']:+.2f} mm. Level floor assumed.",.49)
            else:
                label(660,445,"Missing synchronized body / foot observation")
            out.write(panel)
            written += 1
    finally:
        cap.release()
        out.release()
    if not written:
        raise ValueError("No overlapping trial and video frames in this interval")
    print(f"{written} frames written to {args.output}")


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video",type=Path,required=True)
    parser.add_argument("--decoded",type=Path,required=True,help="Output of convert_rwlog_to_csv.py")
    parser.add_argument("--cad",type=Path,default=SOURCE,help="Original walker assembly GLB")
    sync=parser.add_mutually_exclusive_group(required=True)
    sync.add_argument("--video-zero-s",type=float,help="Video time at measurement zero, assuming equal clock rates")
    sync.add_argument("--clock",type=Path,help="LED fit JSON [first_video_ms,last_video_ms,[[slope,intercept_ms],...]]")
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--start-s",type=float,default=0,help="Start in measurement seconds")
    parser.add_argument("--stop-s",type=float,default=float("inf"),help="End in measurement seconds")
    args=parser.parse_args()
    if not 0 <= args.start_s < args.stop_s:
        parser.error("Specify a non-negative start before stop")
    run(args)
