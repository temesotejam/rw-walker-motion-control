"""Check a measured ZERO-to-peak transition against projected foot motion on the floor grid.

The saved table uses the original clips, the per-trial optical/IMU time offsets,
and the registered floor grid from the turn1/turn2 video studies. It contains
ordinary ZERO crossings followed by a same-side accepted peak in the first 12 s.
Endpoints are averaged across +/-50 or 100 ms; whole-interval endpoints use
+/-400 ms and the 800 ms before release for the initial position. Units are small
grid cells along the camera-facing axis and to the right. Marker height and slip
affect floor projections. At approximately 30 fps the video cannot time contact.
After 12 s the turning trial's markers become occluded; the two straighter trials
have already stopped. These conditional counts cannot measure missed crossings
or establish an effect of changing a pulse.
"""
import argparse
import csv
import importlib.util
import json
from pathlib import Path


HERE = Path(__file__).resolve().parent
TABLE = HERE / 'floor_observations.csv'
# Video time at trial zero from each recording's two white body markers versus IMU.
VIDEOS = {
    52400264: ('WIN_20261120_18_36_33_Pro.mp4', 6.936),
    45820669: ('WIN_20261120_19_01_27_Pro.mp4', 7.094),
    239670034: ('WIN_20261120_19_04_42_Pro.mp4', 7.022),
}
FIELDS = ('run', 'kind', 'start_ms', 'end_ms', 'side', 'halfwidth_ms',
          'right_forward_cells', 'left_forward_cells', 'mid_lateral_cells')


def read_csv(path):
    with path.open(encoding='utf-8-sig', newline='') as stream:
        return list(csv.DictReader(stream))


def measure(recordings, videos):
    import numpy as np

    # Reuse the two original studies' optical tracking, per-run registration and grid.
    spec = importlib.util.spec_from_file_location('turn2_floor_video', HERE.parent / '2026-11-20-turn2-trial/video.py')
    video = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(video)
    records = []
    for run, (name, offset) in VIDEOS.items():
        folder = recordings / str(run)
        if json.loads((folder / 'header.json').read_text(encoding='utf8'))['run_start_us'] != run:
            raise ValueError(f'Incorrect recording in {folder}')
        stamps, points = video.prior.tracks(videos / name)
        t = stamps - offset
        grid_shift = np.array(video.REGISTRATION.get(str(run), [0, 0]))
        feet = np.array([[video.prior.floor_point(p - grid_shift) for p in pair] for pair in points[:, 2:]])

        def endpoint(at, halfwidth):
            mask = abs(1000*t - at) < halfwidth
            if not mask.any():
                return np.full((2, 2), np.nan)
            return feet[mask].mean(axis=0)

        def add(kind, start, end, side, halfwidth):
            base = feet[(-.8 < t) & (t < 0)].mean(axis=0) if kind == 'interval' and start == 0 else endpoint(start, halfwidth)
            delta = endpoint(end, halfwidth) - base
            if not np.isfinite(delta).all():
                raise ValueError(f'Missing optical samples for run {run} at {start}..{end}')
            values = [delta[0, 1], delta[1, 1], delta[:, 0].mean()]
            records.append([run, kind, start, end, side, halfwidth, *[round(v, 7) for v in values]])

        peaks = read_csv(folder / 'energy_control_autonomous_peak_events.csv')
        for z in read_csv(folder / 'energy_control_autonomous_zero_cross_events.csv'):
            if z['event_kind'] != 'energy_control_zero_cross':
                continue
            start, side = int(z['zero_cross_time_ms']), int(z['physical_next_peak_side'])
            peak = next((p for p in peaks if int(p['peak_time_ms']) > start), None)
            if not peak or int(peak['physical_peak_side']) != side:
                continue
            end = int(peak['peak_time_ms']) + 25
            for halfwidth in (50, 100):
                if end + halfwidth <= 12000:
                    add('event', start, end, side, halfwidth)
        for start, end in ((0, 4000), (4000, 8000), (8000, 12000), (0, 12000)):
            add('interval', start, end, 0, 400)
    return records


def swing_summary(rows, side):
    events = [r for r in rows if int(r['side']) == side]
    matched = positive = 0
    for r in events:
        right, left = float(r['right_forward_cells']), float(r['left_forward_cells'])
        swing, other = (right, left) if side == -1 else (left, right)
        matched += swing > other
        positive += swing > other and swing > 0
    return {'events': len(events), 'relative_advance': matched,
            'relative_advance_and_positive_displacement': positive}


def run() -> dict:
    rows = read_csv(TABLE)
    results = {}
    for halfwidth in (50, 100):
        subset = [r for r in rows if r['kind'] == 'event' and int(r['halfwidth_ms']) == halfwidth]
        results[str(halfwidth)] = {
            group: {'leftward_zero_right_foot': swing_summary(sample, -1),
                    'rightward_zero_left_foot': swing_summary(sample, 1)}
            for group, sample in [('all', subset)] + [
                (str(run), [r for r in subset if int(r['run']) == run]) for run in VIDEOS]}
    intervals = []
    for r in rows:
        if r['kind'] == 'interval':
            intervals.append({'run': int(r['run']), 'start_ms': int(r['start_ms']), 'end_ms': int(r['end_ms']),
                              'mid_forward_cells': round((float(r['right_forward_cells'])+float(r['left_forward_cells']))/2, 4),
                              'mid_lateral_cells': round(float(r['mid_lateral_cells']), 4)})
    return {'end_after_matching_accepted_peak_ms': 25, 'max_video_window_ms': 12000,
            'video_time_at_trial_zero_s': {str(k): v[1] for k, v in VIDEOS.items()},
            'by_endpoint_halfwidth_ms': results, 'floor_intervals': intervals}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--measure', nargs=2, type=Path, metavar=('CONVERTED_RUNS', 'VIDEO_DIR'),
                        help='Reconstruct the CSV using the original clips and calibrated tracking')
    args = parser.parse_args()
    if args.measure:
        import sys
        measurements = measure(*args.measure)
        output = csv.writer(sys.stdout, lineterminator='\n')
        output.writerow(FIELDS)
        output.writerows(measurements)
    else:
        print(json.dumps(run(), indent=2, allow_nan=False))
