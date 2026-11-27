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
import aid_current_delivery
import phase_dynamics
import post_zero_carry
import receiving_carry
import return_history_replay
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
    motion = receiving_carry.CarryReplay(trial)
    return {'run': run, 'training_runs': list(return_model.PRIOR),
            'positive_peaks': counts[(3., 'positive_peak')],
            'peaks_missing_each_section': {str(int(s)): counts[(s, 'no_section')] for s in return_model.SECTIONS},
            'evaluation': return_model.evaluate_episodes(reference, test, peaks),
            'early_watch': add_early_watch.evaluate_new(root, trial),
            'action_forecast': phase_dynamics.evaluate_trial(root, trial),
            'observed_return_history': return_history_replay.observed_intervention(trial),
            'observed_return_prediction': return_history_replay.observed_prediction(trial),
            'receiving_carry': receiving_carry.evaluate_trial(trial, motion),
            'first_receiving_outcomes': receiving_carry.first_receiving_outcomes(motion),
            'aid_current_delivery': aid_current_delivery.evaluate_trial(trial, motion),
            'post_zero': post_zero_carry.evaluate_trial(root, trial)}


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
    recordings = {i:phase_dynamics.PhaseRecording(root / str(i)) for i in return_model.PRIOR + return_model.RECENT}
    training = [recordings[i] for i in return_model.PRIOR]
    model = phase_dynamics.PhaseFit([phase_dynamics.training_windows(r) for r in training], phase_dynamics.SELECTED)
    original = action.Fit([action.samples(r) for r in training])
    carry = {i:receiving_carry.CarryReplay(root / str(i)) for i in return_model.RECENT}
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
        plan = carry[rec.run].endpoint(ep['peak_ms'], when)
        review = {'run': rec.run, 'peak_ms': ep['peak_ms'], 'warning_ms': when,
                  'at_warning': at_warning, 'recorded_local_outcome': action.observed(rec, when, terminal),
                  'receiving_plan': plan, 'recorded_receiving_outcome': carry[rec.run].follow(plan),
                  'recorded_return_outcome': ep['outcome'], 'recorded_return_end_ms': ep['end_ms']}
        if plan['terminal_deg'] is not None:
            toward = action.compare(rec, when, model, examples, (0, 10, 20), (0, 25), plan['terminal_deg'])
            if toward['status'] == 'ok':
                toward['observed_followup'] = phase_dynamics.zero_followup(
                    rec, when, ep['peak_ms'], toward['observed_followup'])
            review['toward_receiving_endpoint'] = toward
        review['coast_to_zero'] = {}
        if at_warning['status'] == 'ok':
            coast = action.compare(rec, when, model, examples, (0,), (0,), 0.)
            review['coast_to_zero'] = {
                'previous':action.compare(rec, when, original, examples, (0,), (0,), 0.)['scenarios'][0],
                'common_body_model':coast['scenarios'][0], 'selected':coast['coast_forecast']}
        # A warning after a rescue may call for moving the original input. An additional
        # rescue would be a different intervention, frequently barred by the firmware.
        prior = [p for p in rec.aids if p.peak == ep['peak_ms'] and p.start <= when]
        if prior:
            pulse = prior[-1]
            anchor = pulse.decision - action.STEP
            delay = pulse.start - anchor
            review['retrospective_timing_comparison'] = {
                'recorded_start_ms': pulse.start, 'recorded_width_ms': pulse.width,
                'common_start_state_ms': anchor, 'recorded_delay_from_anchor_ms': delay,
                'comparison': action.compare(rec, anchor, model, examples, (0, pulse.width),
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
        results['action_forecast'] = phase_dynamics.run(root)
    if args.part in ('all', 'carry'):
        results['receiving_carry'] = receiving_carry.run(root)
        results['post_zero'] = post_zero_carry.run(root)
    if args.part in ('all', 'feet'):
        import foot_phase_check
        results['foot_motion'] = foot_phase_check.run(root)
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
        history = trial['observed_return_history']
        measured = trial.get('observed_return_prediction')
        if measured is not None:
            print('  支持移行予測と仕事補正の記録（' + ('全件' if measured['archive_complete'] else '部分記録') + '）')
            for e in measured['events']:
                print(f"    {e['decision_ms']} ms  先読み開始 {e['predicted_support']}  移行前の仕事補正 {e['forecast_before_transfer']}  "
                      f"幅延長 {e['width_extended']}  指令幅 {e['guarded_ms']} ms  実通電 {e['executed']}  "
                      f"延長指令の実通電 {e['executed_extension']}")
                if 'post_aid' in e:
                    post = e['post_aid']
                    print(f"      初回補助の終了 {post['first_end_ms']} ms、再判断 {e['decision_ms']} ms、"
                          f"追加の指令幅 {post['planned_width_ms']} ms、実通電 {e['executed']}")
                if 'passage' in e:
                    allocation = e['passage']
                    print(f"      中心前の補助 {e['base_ms']}→{e['selected_ms']} ms、"
                          f"中心後の予約 {allocation['previous_reserve_ms']}→{allocation['selected_reserve_ms']} ms、"
                          f"予測した中心速度 {allocation['predicted_zero_speed_dps']} °/s")
                if (e['executed_extension'] or 'post_aid' in e) and e.get('motion'):
                    motion = e['motion']
                    zero = motion.get('mechanical_zero') or {}
                    receiving = next((r for r in trial['first_receiving_outcomes']
                                      if r['peak_ms'] == motion['departure_peak_ms']), {})
                    outcomes = []
                    for label, result_at, key in (('中心', zero, 'crossing_speed_dps'),
                                                  ('左円弧端', receiving, 'inward_speed_dps')):
                        speed = result_at.get(key)
                        detail = f" ({speed:.2f} °/s)" if speed is not None else ""
                        outcomes.append(f"{label} {result_at.get('status', '欠測')}{detail}")
                    print("      実測：" + "、".join(outcomes))
        if history['profile'] in ('left_return_history_v1', 'left_return_prediction_v1', 'left_transfer_carry_v1', 'left_transfer_carry_v2', 'left_transfer_carry_v3', 'left_transfer_carry_v4', 'left_transfer_carry_v5'):
            if history['status'] == 'complete':
                print(f"  左補助を評価 {history['evaluated']} 件、過去2回と一致 {history['matching_donors']} 件、"
                      f"延長を選択 {history['width_increased']} 件、延長指令を実行 {history['executed_increased']} 件")
            else:
                print("  左補助の履歴は一部未収録のため、試行全体の件数は未確定です。")
            change = history['first_executed_change']
            if change:
                elapsed = f"{change['elapsed_ms']} ms" if change['elapsed_ms'] is not None else '未確定'
                print(f"  記録内で最初の延長指令 {change['start_ms']} ms、"
                      f"{change['base_ms']} → {change['selected_ms']} ms、実経過 {elapsed}")
        early = {e['peak_ms']: e['early_warning_ms'] for e in trial['early_watch']['episodes']}
        for ep in trial['evaluation']['episodes']:
            warning = ep['warning_ms']
            early_ms = early.get(ep['peak_ms'])
            print(f"  右ピーク {ep['peak_ms']} ms  早期監視 {str(early_ms) + ' ms' if early_ms is not None else 'なし'}"
                  f"  通常警報 {str(warning) + ' ms' if warning is not None else 'なし'}"
                  f"  実測 {ep['outcome']}  評価 {ep['score_status']}")
        summary = receiving_carry.summary(trial['receiving_carry'])['3.0']
        print(f"  +3°から受け足の端まで　終端の記録 {summary['with_receiving_end']} 件、"
              f"無追加通電で採点 {summary['coast_evaluable']} 件、5°/s以上で到達 {summary['coast_carries']} 件")
        after = trial['post_zero']
        delivery = trial['aid_current_delivery']
        complete = [e for e in delivery if e.get('charge_status') == 'complete']
        print(f"  左補助の実測電流　通電中と終了後の両区間を比較できたのは {len(complete)}/{len(delivery)} 件")
        print(f"  左向きZERO通過後　終端を実測 {after['observable_endpoints']} 件、"
              f"別走の類似例で速度予測を採点できたのは {after['supported_scored']} 件")
        for group, label in (('aid', '実補助'), ('coast', '惰行')):
            m = trial['action_forecast']['metrics'][group][phase_dynamics.SELECTED]
            if m['cases']:
                print(f"  {label}の速度予測　評価 {m['cases']} 件、実測と終点の種類が一致 {m['same_endpoint_cases']} 件、"
                      f"速度の平均絶対誤差 {m['speed_mae_dps']} °/s")
    if 'early_watch' in result:
        m = result['early_watch']['prior_leave_one_run_out']['comparison']
        print(f"早期監視 +4.5°を加えた事後比較  同じ{m['reversed']}折返しで検出 "
              f"{m['two']['detected_reversals']}→{m['three']['detected_reversals']}、"
              f"{m['crossed']}通過で誤警報 {m['two']['false_warnings']}→{m['three']['false_warnings']}")
    if 'action_forecast' in result:
        for split, label in (('prior', '先行'), ('held', '直近')):
            m = result['action_forecast'][split]['metrics']
            for group, mode in (('aid', '補助'), ('coast', '惰行')):
                before, after = m[group]['previous'], m[group][phase_dynamics.SELECTED]
                print(f"{label}の{mode} {after['cases']}件　速度の平均絶対誤差 "
                      f"{before.get('speed_mae_dps')} → {after.get('speed_mae_dps')} °/s")
            audit = result['action_forecast'][split]['coast_observer']
            selected = audit[phase_dynamics.SELECTED]
            print(f"{label}の惰行予測　直前の減速を補正した区間 {audit['with_correction']}/{audit['scored']}、"
                  f"終端一致 {selected['correct_endpoints']}、"
                  f"150 ms以内の折返し検出 {selected['detected_reversals']}/{audit['recorded_reversals']}、"
                  f"折返し誤警報 {selected['false_reversals_within_local_horizon']}")
    if 'receiving_carry' in result:
        for split in ('prior', 'recent'):
            counts = result['receiving_carry'][split]['summary']['3.0']
            print(f"{split} +3°判定時に受け側の終端あり {counts['with_receiving_end']}/{counts['decisions']} 件、"
                  f"無追加通電で採点 {counts['coast_evaluable']} 件、5°/s以上で到達 {counts['coast_carries']} 件")
    if 'foot_motion' in result:
        m = result['foot_motion']['held_out']['camera_angle_at_exposure']['scores']['both']
        print(f"直近の足角 {m['n']} 観測　過去値保持の誤差 {m['hold_mae_deg']}°、機上補償 {m['firmware_mae_deg']}°")
    if 'post_zero' in result:
        m = result['post_zero']['prior_leave_one_run_out']
        print(f"左向きZERO通過後の支持　近い別走あり {m['supported_scored']}/{m['observable_endpoints']} 件、"
              f"定速とエネルギー差による速度予測の誤差 {m['constant_speed_mae_dps']:.2f} → {m['empirical_energy_mae_dps']:.2f} °/s")
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
    mode.add_argument('--part', choices=('all', 'returns', 'actions', 'right', 'floor', 'carry', 'feet'), default='all',
                      help='Models to compare on the eight earlier and two later runs')
    mode.add_argument('--evaluate-run', type=Path, help='Score another raw RWLOG or converted directory on the fixed eight runs')
    mode.add_argument('--at', nargs=2, type=int, metavar=('RUN', 'TIME_MS'),
                      help='Compare leftward actions from one recorded state in the ten runs')
    parser.add_argument('--widths', nargs='+', type=int, help='What-if pulse widths in ms, including 0 for coasting')
    parser.add_argument('--delays', nargs='+', type=int, help='What-if start delays from the chosen state in ms')
    parser.add_argument('--section-deg', type=float, help='Requested angle for --at, e.g. 0 for ZERO. Predictions stop at the training boundary at ZERO.')
    parser.add_argument('--json-out', type=Path, help='Save full events and matched examples to this JSON file')
    args = parser.parse_args()
    if any(x is not None for x in (args.widths, args.delays, args.section_deg)) and args.at is None:
        parser.error('--widths, --delays and --section-deg require --at RUN TIME_MS')
    if args.at and (args.at[0] not in return_model.PRIOR + return_model.RECENT or args.at[1] < 0):
        parser.error('--at requires a reference or evaluation run ID and a nonnegative time in milliseconds')
    if any(not 0 <= n <= 80 for n in args.widths or []) or any(not 0 <= n <= 150 for n in args.delays or []):
        parser.error('Use widths of 0..80 ms and delays of 0..150 ms')
    if args.section_deg is not None and (not math.isfinite(args.section_deg) or args.section_deg < -15):
        parser.error('--section-deg must be finite and at least -15 degrees')
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
                carry = receiving_carry.CarryReplay(root / str(args.at[0]))
                plan = carry.at(args.at[1])
                target = args.section_deg if args.section_deg is not None else plan['terminal_deg']
                scenario = phase_dynamics.simulate_at(root, *args.at, widths=tuple(args.widths or (0, 10, 20)),
                          delays=tuple(args.delays or (0, 25)), section_deg=target)
                if scenario['status'] == 'ok':
                    source = 'explicit' if args.section_deg is not None else (
                        'recorded_receiving_anchor' if target is not None else 'local_travel')
                    scenario.update(endpoint_source=source, receiving_plan=plan, recorded_receiving_outcome=carry.follow(plan))
                result = {'scenario': scenario}
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
