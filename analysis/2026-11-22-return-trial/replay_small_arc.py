"""Isolate the last two eligible camera slots of run 195727101 in the production observer.

Run with the standard converter's output folder. The input gate is assumed active.
For shadow snapshots we use the archived capture time as a fresh IMU time proxy:
the archive gives the derivative and fresh-IMU gate, not the exact sample time.
For the intermediate observations we use every distinct archived gyro sample and
its logged age. Samples recorded on disk need not cover all firmware iterations.
"""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import tempfile


HERE = Path(__file__).resolve().parent


def rows(folder, name):
    with (folder / (name + '.csv')).open(encoding='utf8', newline='') as stream:
        return list(csv.DictReader(stream))


def observed_inputs(folder):
    header = json.loads((folder / 'header.json').read_bytes())
    meta = json.loads((folder / 'metadata.json').read_bytes())
    if header['run_start_us'] != 195727101 or meta['walking_profile'] != 'left_return_history_v1':
        raise ValueError('Expected the recorded return-history trial 195727101')
    peak = next(p for p in meta['energy_control_autonomous_peak_events'] if p['peak_time_ms'] == 3739)
    frames = [f for f in meta['foot_frames'] if f['state_id'] == 3 and f['control_observed_us']]
    epochs = {f['frame_us'] - f['measurement_time_us'] for f in frames if f['timestamp_valid']}
    if len(epochs) != 1:
        raise ValueError('The trial camera and measurement times do not have a unique offset')
    epoch, = epochs
    energy = meta['walking_energy']['records']
    current_target = max((r for r in energy if r['time_ms'] < peak['peak_time_ms']), key=lambda r:r['time_ms'])['target_j']
    settings = (header['run_id'], epoch, peak['peak_time_ms'], peak['peak_amplitude_deg'], current_target)
    packets = {int(p['slot_us']):p for p in rows(folder, 'foot_periodic')}
    shadows = {int(s['packet']):s for s in rows(folder, 'support_transfer_shadow') if int(s['peak_ms']) == settings[2]}

    def input_row(s, at_us, sample_us, now_ms, angle, rate, acceleration):
        number = int(s['packet'])
        slot = packets[number * meta['foot_periodic']['period_us']]
        raw = max((f for f in frames if f['control_observed_us'] <= at_us), key=lambda f:f['control_observed_us'])
        if not all(raw[k] for k in ('timestamp_valid', 'frame_valid', 'zero_ready', 'left_valid',
                                    'right_valid', 'left_control_range_ok', 'right_control_range_ok')):
            raise ValueError('Image does not satisfy the fixture validity assumptions')
        if slot['left_sequence'] != slot['right_sequence']:
            raise ValueError('The recorded periodic estimates use different source images')
        image = next(f for f in frames if f['sequence'] == int(slot['left_sequence']))
        return (now_ms, sample_us, at_us - sample_us, angle, rate, acceleration, s['fore_aft_deg'],
                number, epoch + int(slot['slot_us']), epoch + int(slot['publish_us']), image['sequence'],
                image['frame_us'], slot['left_observed_deg'], slot['right_observed_deg'], raw['sequence'],
                raw['frame_us'], (raw['control_observed_us'] - epoch) // 1000, raw['left_deg'], raw['right_deg'])

    def camera_slot(number):
        s = shadows[number]
        t = int(s['capture_us32'])
        assert int(s['gate_mask']) == int(s['baseline_flags']) == 0
        tick = input_row(s, t, t, int(s['time_ms']), s['lateral_deg'], s['rate_dps'], s['accel_dps2'])
        assert tick[10] == tick[14]  # The observer's current raw image matches this published pair.
        return tick

    first, second = camera_slot(160), camera_slot(161)
    inter = {}
    for r in rows(folder, 'timeseries'):
        if r['state_id'] != '3' or not r['body_rate_sample_time_us'] or not r['imu_sample_age_us']:
            continue
        sample = int(r['body_rate_sample_time_us'])
        age = int(r['imu_sample_age_us'])
        at = sample + age
        # Include only observations acquired after the first snapshot and available before the next.
        if first[1] < sample and at < second[1] and sample not in inter:
            speed = (float(r['gy_dps']) - float(r['mekf_bias_y_dps'])) * meta['mekf_gyro_y_scale']
            inter[sample] = input_row(shadows[160], at, sample, int(r['t_test_ms']),
                                      r['pitch_mekf_detector_relative_deg'], speed, 0)
    return settings, first, second, [inter[t] for t in sorted(inter)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('converted', type=Path)
    args = parser.parse_args()
    settings, first, second, intermediate = observed_inputs(args.converted)
    slow = [r for r in intermediate if 0 < -r[4] < 3]
    assert slow and 2.7 < -slow[0][4] < 2.8
    print('Intermediate archived IMU (run ms, control rate deg/s):',
          [(r[0], round(r[4], 3)) for r in intermediate])
    variants = [('slots_only', [], True), ('one_measured_dip', slow[:1], False),
                ('all_archived_intermediate', intermediate, False)]
    with tempfile.TemporaryDirectory() as tmp:
        exe = Path(tmp) / 'small_arc_observer.exe'
        subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                        '-I', HERE.parents[1] / 'src', HERE / 'small_arc_observer.cpp', '-o', exe], check=True)
        for label, between, expected in variants:
            stream = '\n'.join(' '.join(map(str, row)) for row in [settings, first, *between, second]) + '\n'
            result = subprocess.check_output([exe], input=stream, text=True).splitlines()
            t, selected, evidence, work = result[-1].split()
            assert len(result) == len(between) + 2 and t == '4028' and bool(int(selected)) == expected
            assert not expected or int(evidence) == 5
            print(f'{label}: selected={selected}, requested_uJ={float(work):.3f}')


if __name__ == '__main__':
    main()
