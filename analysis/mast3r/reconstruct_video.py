"""Export MASt3R frame-pair predictions and reciprocal matches from a local video."""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import sys

from install import DEFAULT_RUNTIME


def extract_frames(video, times, output):
    import cv2
    capture = cv2.VideoCapture(str(video))
    if not capture.isOpened():
        raise ValueError(f"Cannot open local video: {video}")
    frames = []
    previous = None
    index = 0
    try:
        fps = capture.get(cv2.CAP_PROP_FPS)
        while len(frames) < len(times):
            ok, image = capture.read()
            if not ok:
                break
            pts = capture.get(cv2.CAP_PROP_POS_MSEC) / 1000
            clock = "decoded_pts"
            if not math.isfinite(pts) or (previous and pts <= previous[1]):
                if not math.isfinite(fps) or fps <= 0:
                    raise ValueError("No usable timestamp or declared frame rate")
                pts = index / fps
                clock = "nominal_fps"
                if previous and pts <= previous[1]:
                    raise ValueError("Inconsistent video clock")
            current = index, pts, image, clock
            while len(frames) < len(times) and pts >= times[len(frames)]:
                target = times[len(frames)]
                item = previous if previous and target-previous[1] < pts-target else current
                if frames and item[0] == frames[-1]["frame_index"]:
                    raise ValueError("Two requested times selected the same source frame")
                name = f"frame-{item[0]:06d}.png"
                encoded, buffer = cv2.imencode(".png", item[2])
                if not encoded:
                    raise OSError(f"Unable to encode {name}")
                (output / name).write_bytes(buffer.tobytes())
                frames.append(dict(requested_s=target, video_s=item[1], clock=item[3],
                                   frame_index=item[0], image=name, width=item[2].shape[1],
                                   height=item[2].shape[0]))
            previous = current
            index += 1
    finally:
        capture.release()
    if len(frames) != len(times):
        raise ValueError("A requested timestamp lies beyond the last decoded video frame")
    return frames


def write_ply(path, xyz, rgb):
    import numpy as np
    vertices = np.empty(len(xyz), dtype=[(k, "<f4") for k in ("x", "y", "z")] +
                                        [(k, "u1") for k in ("red", "green", "blue")])
    for i, k in enumerate(("x", "y", "z")):
        vertices[k] = xyz[:, i]
    for i, k in enumerate(("red", "green", "blue")):
        vertices[k] = rgb[:, i]
    header = (f"ply\nformat binary_little_endian 1.0\nelement vertex {len(xyz)}\n"
              "property float x\nproperty float y\nproperty float z\nproperty uchar red\n"
              "property uchar green\nproperty uchar blue\nend_header\n")
    with path.open("wb") as stream:
        stream.write(header.encode("ascii"))
        vertices.tofile(stream)


def region_mask(frame, shape, roi, mask_path, crop=None):
    import numpy as np
    from PIL import Image
    h, w = shape
    if mask_path:
        with Image.open(mask_path) as mask:
            original = frame["width"], frame["height"]
            if mask.size != original:
                raise ValueError(f"Mask dimensions must match original frame: {mask_path}")
            if crop:
                mask = mask.crop(crop)
            original = mask.size
            resized = tuple(round(v*512/max(original)) for v in original)
            x, y = resized[0]//2-w//2, resized[1]//2-h//2
            mask = mask.convert("L").resize(resized, Image.Resampling.NEAREST)
            return np.asarray(mask.crop((x, y, x+w, y+h))) > 127
    if roi:
        y, x = np.ogrid[:h, :w]
        return ((x+.5 >= roi[0]*w) & (x+.5 < roi[2]*w) &
                (y+.5 >= roi[1]*h) & (y+.5 < roi[3]*h))
    return None


def export_pair(path, result, pixels, masks, min_conf):
    import numpy as np
    from PIL import Image
    path.mkdir()
    dense = []
    matches_ok = np.ones(len(pixels[0]), dtype=bool)
    for i, (pts_key, xy) in enumerate(zip(("pts3d", "pts3d_in_other_view"), pixels)):
        pred = result[f"pred{i+1}"]
        xyz = pred[pts_key][0].numpy()
        conf = pred["conf"][0].numpy()
        img = result[f"view{i+1}"]["img"][0].permute(1, 2, 0).numpy()
        rgb = np.uint8(np.clip((img+1)*127.5, 0, 255).round())
        Image.fromarray(rgb).save(path / f"view{i}.png")
        ok = np.isfinite(xyz).all(axis=2) & np.isfinite(conf) & (conf >= min_conf) & (xyz[:,:,2]>0)
        # Keep image indexing for independent CAD / floor checks and threshold sensitivity.
        np.savez_compressed(path / f"view{i}-pointmap.npz", xyz=xyz, confidence=conf, accepted=ok)
        u, v = xy.T
        h, w = ok.shape
        matches_ok &= ok[v, u] & (u>=3) & (v>=3) & (u<w-3) & (v<h-3)
        dense.append((xyz, rgb, conf, ok))
    def store(name, selections):
        points = np.concatenate([xyz[sel] for (xyz, rgb, conf, ok),sel in zip(dense,selections)])
        colors = np.concatenate([rgb[sel] for (xyz, rgb, conf, ok),sel in zip(dense,selections)])
        write_ply(path / name, points, colors)
        return len(points)
    counts = {"dense":store("dense.ply", [v[3] for v in dense])}
    if masks[0] is not None:
        for name, inside in (("roi", True), ("outside_roi", False)):
            counts[name] = store("dense_"+name+".ply", [v[3] & (m==inside) for v,m in zip(dense,masks)])
    xy0, xy1 = (p[matches_ok] for p in pixels)
    u0,v0 = xy0.T
    u1,v1 = xy1.T
    p0, p1 = dense[0][0][v0,u0], dense[1][0][v1,u1]
    write_ply(path/"sparse.ply",p0,dense[0][1][v0,u0])
    groups = np.full(len(p0), "unlabelled", dtype="U12")
    if masks[0] is not None:
        a, b = masks[0][v0,u0], masks[1][v1,u1]
        groups[:] = "mixed"
        groups[a & b] = "roi"
        groups[~a & ~b] = "outside_roi"
    with (path/"matches.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["u0","v0","u1","v1","x0","y0","z0","x1","y1","z1","conf0","conf1","region"])
        for j, group in enumerate(groups):
            w.writerow([*xy0[j],*xy1[j],*p0[j],*p1[j],dense[0][2][v0[j],u0[j]],dense[1][2][v1[j],u1[j]],group])
    stats = {}
    for group in np.unique(groups):
        selected = groups==group
        delta = np.linalg.norm(xy1[selected]-xy0[selected],axis=1)
        stats[str(group)] = dict(count=int(selected.sum()), median_image_motion_px=float(np.median(delta)))
    return dict(points=counts, reciprocal_matches=len(p0), match_regions=stats)


def predict(args, frames):
    import torch
    if args.device == "cuda" and not torch.cuda.is_available():
        raise ValueError("CUDA unavailable; use --device cpu or a CUDA PyTorch environment")
    if not (args.source/"mast3r/model.py").is_file() or not (args.source/"dust3r/croco/models/croco.py").is_file():
        raise ValueError("Missing official source or its recursive submodules. Run install.py.")
    if not args.weights.exists():
        raise ValueError("Missing local checkpoint. Run install.py or supply --weights.")
    sys.path.insert(0, str(args.source.resolve()))
    from mast3r.model import AsymmetricMASt3R
    from mast3r.fast_nn import fast_reciprocal_NNs
    from dust3r.utils.image import load_images
    from dust3r.inference import inference
    source = args.source.resolve()
    revision = subprocess.check_output(["git","-c","safe.directory="+source.as_posix(),"-C",str(source),
                                        "rev-parse","HEAD"],text=True).strip()
    model = AsymmetricMASt3R.from_pretrained(str(args.weights.resolve())).eval().to(args.device)
    paths = [args.output / f["image"] for f in frames]
    if args.crop:
        from PIL import Image
        for i, path in enumerate(paths):
            with Image.open(path) as original:
                paths[i] = path.with_name(path.stem+"-crop.png")
                original.crop(args.crop).save(paths[i])
    images = load_images([str(path) for path in paths], size=512)
    masks = [region_mask(f, im["img"].shape[-2:], args.roi, args.masks[i] if args.masks else None, args.crop)
             for i, (f, im) in enumerate(zip(frames, images))]
    for frame, image in zip(frames, images):
        x0,y0,x1,y1 = args.crop or (0,0,frame["width"],frame["height"])
        factor = 512 / max(x1-x0,y1-y0)
        rw,rh = round((x1-x0)*factor), round((y1-y0)*factor)
        h,w = image["img"].shape[-2:]
        frame["network_grid"] = dict(width=w, height=h, resized_width=rw, resized_height=rh,
                                    center_crop_x=rw//2-w//2, center_crop_y=rh//2-h//2,
                                    source_box_xyxy=[x0,y0,x1,y1])
    pairs = []
    for i in range(len(images)-1):
        result = inference([(images[i],images[i+1])], model, args.device, batch_size=1)
        descriptors = [result[key]["desc"][0] for key in ("pred1","pred2")]
        pixels = fast_reciprocal_NNs(*descriptors, subsample_or_initxy1=8, device=args.device, dist="dot", block_size=2048)
        folder = f"pair-{i:02d}-{i+1:02d}"
        pair = export_pair(args.output/folder,result,pixels,masks[i:i+2],args.min_conf)
        pairs.append(dict(folder=folder, frames=[f["frame_index"] for f in frames[i:i+2]], **pair))
    return dict(source_commit=revision, torch=str(torch.__version__), device=args.device,
                min_point_confidence=args.min_conf, pair_coordinates="separate first-view model coordinates", pairs=pairs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--times", required=True, nargs="+", type=float, help="Strictly increasing video seconds")
    parser.add_argument("--source", type=Path, default=DEFAULT_RUNTIME/"source")
    parser.add_argument("--weights", type=Path, default=DEFAULT_RUNTIME/"model")
    parser.add_argument("--device", choices=("cpu","cuda"), default="cpu")
    parser.add_argument("--crop", nargs=4, type=int, metavar=("X0","Y0","X1","Y1"),
                        help="Crop original frames before model resizing. Keep the same box for every frame.")
    parser.add_argument("--prepare-only", action="store_true", help="Test video decoding without weights or PyTorch")
    parser.add_argument("--min-conf", default=1.5, type=float, help="Minimum network point confidence")
    region = parser.add_mutually_exclusive_group()
    region.add_argument("--roi", nargs=4, type=float, metavar=("X0","Y0","X1","Y1"),
                        help="Robot region in normalized model-crop coordinates, shared by both views")
    region.add_argument("--masks", nargs="+", type=Path, help="One original-frame-sized binary robot mask per timestamp")
    args = parser.parse_args()
    if len(args.times)<2 or any(not math.isfinite(t) or t<0 for t in args.times) or any(a>=b for a,b in zip(args.times,args.times[1:])):
        parser.error("Provide at least two strictly increasing finite non-negative timestamps")
    if not math.isfinite(args.min_conf) or args.min_conf<0:
        parser.error("--min-conf must be finite and non-negative")
    if args.roi and not (0<=args.roi[0]<args.roi[2]<=1 and 0<=args.roi[1]<args.roi[3]<=1):
        parser.error("ROI requires 0 <= x0 < x1 <= 1 and 0 <= y0 < y1 <= 1")
    if args.masks and (len(args.masks)!=len(args.times) or not all(p.is_file() for p in args.masks)):
        parser.error("Supply one readable mask for each timestamp")
    if not args.video.is_file():
        parser.error("Local video file missing")
    if args.output.exists() and (not args.output.is_dir() or any(args.output.iterdir())):
        parser.error("Choose a new or empty output directory")
    args.output.mkdir(exist_ok=True,parents=True)
    frames = extract_frames(args.video,args.times,args.output)
    if args.crop:
        x0,y0,x1,y1 = args.crop
        if any(not (0<=x0<x1<=f["width"] and 0<=y0<y1<=f["height"]) for f in frames):
            parser.error("Crop must be a non-empty rectangle within every original frame")
    report = dict(video=args.video.name,frames=frames,roi=args.roi,
                  crop_xyxy=args.crop,
                  masks=[p.name for p in args.masks] if args.masks else None, mode="frames_only")
    manifest = args.output/"summary.json"
    manifest.write_text(json.dumps(report,indent=2,ensure_ascii=False)+"\n",encoding="utf8")
    if not args.prepare_only:
        report.update(predict(args,frames))
        report["mode"] = "pair_predictions"
        manifest.write_text(json.dumps(report,indent=2,ensure_ascii=False)+"\n",encoding="utf8")
    print(manifest)


if __name__ == "__main__":
    main()
