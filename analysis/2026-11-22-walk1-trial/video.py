"""Measure both feet against the floor in the 16:56 clip and align the body to the IMU."""
import argparse
import json
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import cv2
import numpy as np

here = Path(__file__).resolve().parent
spec = spec_from_file_location('floor_measurements', here.parent / '2026-11-22-return-trial/video.py')
shared = module_from_spec(spec)
spec.loader.exec_module(shared)
MARKERS = ((564, 267), (774, 264), (645, 404), (688, 404))
THROUGH_S = 20.


def mat_position(point):
    x, y = point
    # Full horizontal grid lines from the first video frame at x=475 and x=810.
    left = np.array([403,413,423,433,443,455,466,478])
    right = np.array([403,412,422,432,443,454,465,477])
    rows = left + (right-left)*(x-475)/335
    # Fourteen full cells between these two vertical lines.
    interp = shared.optical.interp
    lo = interp(y, [420,443,480], [507,501,493])
    hi = interp(y, [420,443,480], [805,815,829])
    return [14*(x-lo)/(hi-lo), interp(y, rows, np.arange(len(rows)))]


def check_mat(video):
    cap = cv2.VideoCapture(str(video))
    ok, image = cap.read()
    if not ok:
        raise OSError(video)
    reference = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    offsets, scores = [], []
    for second in range(2, 27, 2):
        cap.set(cv2.CAP_PROP_POS_MSEC, 1000*second)
        ok, image = cap.read()
        if not ok:
            raise ValueError(f'No frame at {second} seconds')
        sample = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        for x,y in [(475,445),(827,445),(475,495),(827,495),(520,565),(790,565)]:
            roi = reference[y-15:y+16,x-15:x+16]
            search = sample[y-19:y+20,x-19:x+20]
            _, score, _, at = cv2.minMaxLoc(cv2.matchTemplate(search,roi,cv2.TM_CCOEFF_NORMED))
            scores.append(score)
            offsets.append(np.array(at)-4)
    cap.release()
    drift = int(np.max(np.abs(offsets)))
    # Dark grid patches have less contrast. Require four independently matched
    # regions at each time and check displacement for all six.
    min_strong = int((np.asarray(scores).reshape(-1,6)>=.8).sum(axis=1).min())
    if drift > 1 or min_strong < 4:
        raise ValueError('Check camera registration against the mat')
    return dict(patches=len(scores),maximum_drift_px=drift,minimum_correlation=min(scores),
                minimum_strong_patches_per_frame=min_strong)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('converted',type=Path,help='Converted directory for RWLOG 68907120')
    parser.add_argument('--video',type=Path,default=Path.home()/'Pictures/Camera Roll/WIN_20261122_16_56_51_Pro.mp4')
    args = parser.parse_args()
    mat = check_mat(args.video)
    t, feet, marks, fits = shared.aligned_observations(args.video,args.converted,np.zeros(2),
                                                       MARKERS,mat_position,through_s=THROUGH_S)
    result = shared.measure_positions(args.converted,t,feet,marks,fits,through_s=THROUGH_S)
    meta = json.loads((args.converted/'metadata.json').read_bytes())
    last_aid = [r for r in meta['walking_energy']['records'] if r['return_event'] and r['side']==-1 and r['executed']][-1]
    on = last_aid['output_start_ms']/1000
    final = np.median(feet[(t>=12)&(t<=THROUGH_S),:,1],axis=0)
    at_on = feet[np.abs(t-on)<.05,:,1].mean(axis=0)
    result.update(
        mat=mat,observed_video_frames=len(t),last_video_frame_test_s=t[-1],
        final_aid_start_s=on,remaining_forward_after_last_aid_cells=(final-at_on).tolist())
    print(json.dumps(result,indent=2,allow_nan=False))


if __name__=='__main__':
    main()
