import csv
import tempfile
from pathlib import Path

from convert_rwlog_to_csv import write_energy_control_autonomous_events


def convert(version):
    events = []
    for side in (-1, 1):
        events.append({
            'physical_next_peak_side': side,
            'long_foot_counterfactual_ms': 28,
            'long_foot_actual_ms': 33,
            'long_foot_effective': True,
            'foot_span_flags': 1 << 12,
        })
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory)
        write_energy_control_autonomous_events(
            {'firmware_revision': version, 'energy_control_autonomous_zero_cross_events': events}, path)
        with (path / 'energy_control_autonomous_zero_cross_events.csv').open(newline='', encoding='utf-8') as stream:
            return list(csv.DictReader(stream))


for version in ('1.0.91-two-cycle-recovery', '1.0.92-paired-foot-sequence', '1.0.93-blocked-candidate-retry', '1.0.94-foot-trial-loss', '1.0.95-positive-sway-probe', '1.0.96-tapered-amplitude-recovery', '1.0.97-paired-sway-rollback', '1.0.98-foot-observation-timeout', '1.0.99-alternate-foot-trial'):
    negative, positive = convert(version)
    assert negative['negative_recovery_baseline_ms'] == '28'
    assert negative['negative_recovery_candidate_ms'] == '33'
    assert negative['negative_recovery_executed'] == 'True'
    assert negative['negative_recovery_state'] == '1'
    assert negative['long_foot_actual_ms'] == ''
    assert positive['long_foot_actual_ms'] == '33'
    assert positive['negative_recovery_candidate_ms'] == ''

negative, positive = convert('1.0.90-long-foot-window')
assert negative['long_foot_actual_ms'] == '33'
assert negative['negative_recovery_candidate_ms'] == ''
assert positive['long_foot_actual_ms'] == '33'
print('Foot log side mapping for 1.0.90 through 1.0.95 PASS')
