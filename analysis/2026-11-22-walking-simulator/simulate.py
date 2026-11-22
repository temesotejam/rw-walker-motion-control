"""Replay the 11/20 walking trials or score a new RWLOG using the eight reference runs."""
import argparse
import contextlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile

import add_early_watch
import return_model


HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'tools'))
from convert_rwlog_to_csv import convert


def recording_id(folder):
    return int(json.loads((folder / 'header.json').read_text(encoding='utf-8'))['run_start_us'])


def decode(raw, target):
    # The project decoder checks the full CRC and the binary schema before emitting CSVs.
    with contextlib.redirect_stdout(io.StringIO()):
        convert(raw, target)
    return target


def reference_root(args, temporary, run_ids):
    root = args.recordings or temporary / 'reference'
    for run in run_ids:
        destination = root / str(run)
        if args.recordings is None:
            files = sorted(args.raw_dir.glob(f'*_{run}.rwlog'))
            if len(files) != 1:
                raise ValueError(f'Expected one *_{run}.rwlog in {args.raw_dir}, found {len(files)}')
            decode(files[0], destination)
        if recording_id(destination) != run:
            raise ValueError(f'Run identity mismatch in {destination}')
    return root


def evaluate_new(root, trial):
    run = recording_id(trial)
    if run in return_model.PRIOR:
        raise ValueError(f'{run} is in the training set. Use the built-in leave-one-run-out evaluation.')
    reference = []
    for prior in return_model.PRIOR:
        rows, _ = return_model.load_sections(root / str(prior))
        reference.extend(rows)
    peaks = []
    test, counts = return_model.load_sections(trial, episodes=peaks)
    return {'run': run, 'training_runs': list(return_model.PRIOR),
            'positive_peaks': counts[(3., 'positive_peak')],
            'peaks_missing_each_section': {str(int(s)): counts[(s, 'no_section')] for s in return_model.SECTIONS},
            'evaluation': return_model.evaluate_episodes(reference, test, peaks),
            'early_watch': add_early_watch.evaluate_new(root, trial)}


def write_json(path, result):
    # Validate before opening the destination, and keep an existing file intact on failure.
    payload = json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False)
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                     suffix='.tmp', delete=False) as stream:
        temp = Path(stream.name)
        try:
            stream.write(payload)
            stream.write('\n')
        except BaseException:
            stream.close()
            temp.unlink(missing_ok=True)
            raise
    try:
        temp.replace(path)
    finally:
        temp.unlink(missing_ok=True)


def examine_warnings(root, returns):
    """Compare pulse timing from the same state. Keep retrospective rewinds separate."""
    import action_model as action
    recordings = action.load(root)
    training = [recordings[i] for i in return_model.PRIOR]
    model = action.Fit([action.samples(r) for r in training])
    examples = action.action_examples(training)
    # Reference landmark of the upright sole. The actual load switch can move with the feet.
    nominal_arc_edge = math.degrees(math.asin(5 / 150))
    reviews = []
    for ep in returns['held_out']['sequential']['episodes']:
        when = ep['warning_ms']
        if when is None:
            continue
        observation = next(x for x in ep['observations'] if x['time_ms'] == when and x['warning'])
        terminal = nominal_arc_edge if observation['body_angle_deg'] > nominal_arc_edge else 0.
        rec = recordings[ep['run']]
        at_warning = action.compare(rec, when, model, examples, (0, 10, 20), (0, 25), terminal)
        review = {'run': rec.run, 'peak_ms': ep['peak_ms'], 'warning_ms': when,
                  'at_warning': at_warning, 'recorded_local_outcome': action.observed(rec, when, terminal),
                  'recorded_return_outcome': ep['outcome'], 'recorded_return_end_ms': ep['end_ms']}
        # A warning after a rescue may call for moving the original input. An additional
        # rescue would be a different intervention, frequently barred by the firmware.
        prior = [p for p in rec.aids if p.peak == ep['peak_ms'] and p.start <= when]
        if prior:
            original = prior[-1]
            anchor = original.decision - action.STEP
            delay = original.start - anchor
            review['retrospective_timing_comparison'] = {
                'recorded_start_ms': original.start, 'recorded_width_ms': original.width,
                'common_start_state_ms': anchor, 'recorded_delay_from_anchor_ms': delay,
                'comparison': action.compare(rec, anchor, model, examples, (0, original.width),
                                             (0, delay))}
        reviews.append(review)
    return reviews


def compare(args, root):
    results = {}
    if args.part in ('all', 'returns'):
        results['returns'] = return_model.run(root)
        results['early_watch'] = add_early_watch.run(root)
    if args.part in ('all', 'actions'):
        import action_model
        results['actions'] = action_model.run(root)
    if args.part in ('all', 'right'):
        import right_candidates
        results['right'] = right_candidates.run(root)
    if args.part in ('all', 'floor'):
        import floor_check
        results['floor'] = floor_check.run()
    if args.part == 'all':
        results['warnings'] = examine_warnings(root, results['returns'])
    return results


def print_summary(result):
    if 'scenario' in result:
        print(json.dumps(result['scenario'], indent=2, ensure_ascii=False, allow_nan=False))
        return
    if 'returns' in result:
        for field, label in (('prior_leave_one_run_out', '先行8走・1走ずつ除外'), ('held_out', '直近2走')):
            group = result['returns'][field]['sequential']
            m = group['metrics']['warning']
            print(f"{label}  判定 {group['counts'].get('scored', 0)} 半周期、折返し検出 {m['detected_reversals']}、"
                  f"見逃し {m['missed_reversals']}、誤警報 {m['false_warnings']}")
    if 'new_trial' in result:
        trial = result['new_trial']
        print(f"試行 {trial['run']}  右ピーク {trial['positive_peaks']} 件")
        early = {e['peak_ms']: e['early_warning_ms'] for e in trial['early_watch']['episodes']}
        for ep in trial['evaluation']['episodes']:
            warning = ep['warning_ms']
            early_ms = early.get(ep['peak_ms'])
            print(f"  右ピーク {ep['peak_ms']} ms  早期監視 {str(early_ms) + ' ms' if early_ms is not None else 'なし'}"
                  f"  通常警報 {str(warning) + ' ms' if warning is not None else 'なし'}"
                  f"  実測 {ep['outcome']}  評価 {ep['score_status']}")
    if 'early_watch' in result:
        m = result['early_watch']['prior_leave_one_run_out']['comparison']
        print(f"早期監視 +4.5°を加えた事後比較  同じ{m['reversed']}折返しで検出 "
              f"{m['two']['detected_reversals']}→{m['three']['detected_reversals']}、"
              f"{m['crossed']}通過で誤警報 {m['two']['false_warnings']}→{m['three']['false_warnings']}")
    if 'actions' in result:
        m = result['actions']['metrics']['holdout']
        print(f"直近の実通電  評価 {m['scored']} 件、局所到達速度の平均絶対誤差 {m['matched_speed_mae_dps']} °/s")
    if 'right' in result:
        for row in result['right']['decisions']:
            if row['split'] == 'two_recent_runs' and row['comparison_ms'] is not None:
                print(f"右ZERO {row['run']} / {row['zero_ms']} ms  元の幅 {row['original_ms']} ms、"
                      f"arrival1 {row['arrival1_ms']} ms、近い走行例がある比較幅 {row['comparison_ms']} ms")
    if 'floor' in result:
        for path in result['floor']['floor_intervals']:
            if path['start_ms'] == 0 and path['end_ms'] == 12000:
                print(f"床映像 {path['run']}  最初の12秒の中点前進 {path['mid_forward_cells']:.2f} 小区画")
    for review in result.get('warnings', []):
        print(f"左戻り {review['run']}  警報 {review['warning_ms']} ms、実測折返しまたは中心到達 "
              f"{review['recorded_return_end_ms']} ms、付近の左補助実績 "
              f"{review['at_warning'].get('matched_prior_actions', 0)} 件")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument('--recordings', type=Path, help='Converted CSV root with a numbered directory per run')
    source.add_argument('--raw-dir', type=Path, default=Path.home() / 'Downloads',
                        help='Directory containing original reference RWLOGs (default: Downloads)')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--part', choices=('all', 'returns', 'actions', 'right', 'floor'), default='all',
                      help='Models to compare on the eight earlier and two later runs')
    mode.add_argument('--evaluate-run', type=Path, help='Score another raw RWLOG or converted directory on the fixed eight runs')
    mode.add_argument('--at', nargs=2, type=int, metavar=('RUN', 'TIME_MS'),
                      help='Compare leftward actions from one recorded state in the ten runs')
    parser.add_argument('--widths', nargs='+', type=int, help='What-if pulse widths in ms, including 0 for coasting')
    parser.add_argument('--delays', nargs='+', type=int, help='What-if start delays from the chosen state in ms')
    parser.add_argument('--section-deg', type=float, help='Terminal body angle for --at, e.g. 0 for mechanical ZERO')
    parser.add_argument('--json-out', type=Path, help='Save full events and matched examples to this JSON file')
    args = parser.parse_args()
    if any(x is not None for x in (args.widths, args.delays, args.section_deg)) and args.at is None:
        parser.error('--widths, --delays and --section-deg require --at RUN TIME_MS')
    if args.at and (args.at[0] not in return_model.PRIOR + return_model.RECENT or args.at[1] < 0):
        parser.error('--at requires a reference or evaluation run ID and a nonnegative time in milliseconds')
    if any(not 0 <= n <= 80 for n in args.widths or []) or any(not 0 <= n <= 150 for n in args.delays or []):
        parser.error('Use widths of 0..80 ms and delays of 0..150 ms')
    if args.section_deg is not None and (not math.isfinite(args.section_deg) or args.section_deg < 0):
        parser.error('--section-deg must be a finite nonnegative angle')
    if args.json_out and args.json_out.suffix.lower() != '.json':
        parser.error('--json-out must be a .json file')
    try:
        with tempfile.TemporaryDirectory(prefix='walker_sim_') as tmp:
            temporary = Path(tmp)
            runs = return_model.PRIOR if args.evaluate_run else return_model.PRIOR + return_model.RECENT
            root = reference_root(args, temporary, () if args.part == 'floor' else runs)
            if args.evaluate_run:
                trial = args.evaluate_run
                if trial.is_file():
                    trial = decode(trial, temporary / 'new_trial')
                result = {'new_trial': evaluate_new(root, trial)}
            elif args.at:
                import action_model
                result = {'scenario': action_model.simulate_at(root, *args.at,
                          widths=tuple(args.widths or (0, 10, 20)), delays=tuple(args.delays or (0, 25)),
                          section_deg=args.section_deg)}
            else:
                result = compare(args, root)
            print_summary(result)
            if args.json_out:
                write_json(args.json_out, result)
                print(f'Saved {", ".join(result)} to {args.json_out}')
    except (OSError, ValueError, KeyError) as error:
        parser.exit(1, f'Cannot replay trial: {error}\n')


if __name__ == '__main__':
    main()
