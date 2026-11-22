import csv
import json
from pathlib import Path
import tempfile
import unittest

from return_history_replay import observed_intervention, observed_prediction


class InterventionReport(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.folder = Path(tmp.name)
        epoch = 1000000
        first = dict(time_ms=140, peak_ms=100, side=-1, return_event=True, executed=False, guarded_ms=24, output_start_ms=0)
        second = dict(first, time_ms=148, executed=True, output_start_ms=150, output_end_ms=163,
                      actual_elapsed_ms=13, completion_reason=2, planned_duration_completed=False)
        main = dict(time_ms=190, peak_ms=100, side=-1, return_event=False, executed=True, guarded_ms=30, output_start_ms=192)
        arc = dict(arc_history_first_peak_ms=10, arc_history_second_peak_ms=50, arc_history_base_ms=19,
                   arc_history_selected_ms=24, arc_terminal_deg=-0.8)
        self.meta = dict(walking_profile='left_return_history_v1', mekf_gyro_y_scale=1,
                         walking_energy=dict(archive_available=True, stored=3, total=3, truncated=False,
                                             records=[first, second, main], arc_aids=[dict(arc,row_index=0), dict(arc,row_index=1)]),
                         energy_control_autonomous_peak_events=[
                             dict(peak_time_ms=100,peak_telemetry_capture_time_ms=105,physical_peak_side=1,peak_amplitude_deg=3),
                             dict(peak_time_ms=250,peak_telemetry_capture_time_ms=255,physical_peak_side=-1,peak_amplitude_deg=1.2)],
                         energy_control_autonomous_zero_cross_events=[
                             dict(event_kind='energy_control_zero_cross',physical_next_peak_side=-1,
                                  previous_peak_time_ms=100,zero_cross_time_ms=190)],
                         foot_frames=[dict(state_id=3,timestamp_valid=True,frame_valid=True,zero_ready=True,
                                           frame_us=epoch+t*1000,measurement_time_us=t*1000,
                                           left_control_range_ok=True,left_valid=True,left_deg=0.25*i,
                                           right_control_range_ok=True,right_valid=True,right_deg=-0.5*i)
                                      for i,t in enumerate((150,190,240))])
        with (self.folder/'timeseries.csv').open('w',newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['state_id', 'body_rate_sample_time_us','gy_dps','mekf_bias_y_dps','pitch_mekf_detector_relative_deg'])
            for t in range(145, 256, 5):
                writer.writerow([3,epoch+t*1000,-20,0,(190-t)*0.02])

    def read(self):
        (self.folder/'metadata.json').write_text(json.dumps(self.meta),encoding='utf8')
        return observed_intervention(self.folder)

    def test_actual_width_and_following_main_are_separate(self):
        result = self.read()
        self.assertEqual((result['width_increased'],result['executed_increased']),(2,1))
        first = result['first_executed_change']
        self.assertEqual((first['start_ms'],first['selected_ms'],first['elapsed_ms']), (150,24,13))
        self.assertFalse(first['completed_as_planned'])
        self.assertFalse(first['elapsed_exceeded_base'])
        self.assertEqual(first['mechanical_zero']['event_ms'],190)
        self.assertEqual(first['receiving_end']['event_ms'],230)
        self.assertEqual(first['left_peak_deg'],-1.2)
        self.assertEqual(first['subsequent_commands_observed'],[dict(start_ms=192,side=-1,width_ms=30)])
        self.assertEqual(first['observed_foot_angles']['left']['excursion_deg'],0.5)

    def test_new_profile_distinguishes_veto_from_executed_width_change(self):
        self.meta['walking_profile'] = 'left_return_prediction_v1'
        rows = self.meta['walking_energy']['records']
        arcs = self.meta['walking_energy']['arc_aids']
        rows[0]['return_evidence'] = 6
        for a in arcs:
            a.update(arc_predicted_transfer_ms=None, arc_predicted_terminal_deg=None,
                     arc_crossing_ref_zero_ms=0, arc_crossing_base_ms=0, arc_crossing_selected_ms=0)
        arcs[0].update(arc_predicted_transfer_ms=28.5, arc_predicted_terminal_deg=-0.85)
        arcs[1].update(arc_crossing_ref_zero_ms=50, arc_crossing_base_ms=19, arc_crossing_selected_ms=24)
        self.read()
        result = observed_prediction(self.folder)
        self.assertTrue(result['archive_complete'])
        veto, extended = result['events']
        self.assertTrue(veto['predicted_support'])
        self.assertFalse(veto['executed'])
        self.assertIsNone(veto['motion'])
        self.assertTrue(extended['executed'] and extended['width_extended'])
        self.assertEqual(extended['motion']['mechanical_zero']['event_ms'],190)
        self.assertEqual(extended['reference_zero_ms'],50)
        self.meta['walking_energy']['truncated'] = True
        self.read()
        self.assertFalse(observed_prediction(self.folder)['archive_complete'])

    def test_unknown_totals_on_truncation_and_missing_history(self):
        self.meta['walking_energy']['truncated'] = True
        result = self.read()
        self.assertIsNone(result['executed_increased'])
        self.assertEqual(result['observed_minimum']['executed_increased'],1)
        self.meta['walking_energy'].pop('arc_aids')
        self.assertIsNone(self.read()['observed_minimum'])
        self.meta.pop('walking_profile')
        self.assertEqual(self.read()['status'],'history_not_logged')

    def test_forecasted_endpoint_and_executed_extension_on_carry_profile(self):
        self.meta['walking_profile'] = 'left_transfer_carry_v1'
        for a in self.meta['walking_energy']['arc_aids']:
            a.update(arc_predicted_terminal_deg=None, arc_predicted_transfer_ms=None, arc_terminal_deg=None,
                     arc_approach_transfer_ms=38.8, arc_approach_terminal_deg=-0.8, arc_approach_request_j=3.5e-5,
                     arc_crossing_request_j=8.1e-5, arc_crossing_ref_zero_ms=40,
                     arc_crossing_base_ms=19, arc_crossing_selected_ms=24)
        self.read()
        result = observed_prediction(self.folder)
        self.assertTrue(result['archive_complete'])
        veto, sent = result['events']
        self.assertFalse(veto['executed_extension'])
        self.assertTrue(sent['forecast_before_transfer'] and sent['executed_extension'])
        self.assertFalse(sent['predicted_support'] or sent['elapsed_exceeded_base'])
        self.assertEqual((sent['transfer_in_ms'], sent['arc_work_j'], sent['combined_work_j']), (38.8,3.5e-5,8.1e-5))
        self.assertEqual(sent['motion']['receiving_end']['event_ms'],230)
        # A completed forecast can leave the original duration in place.
        self.meta['walking_energy']['arc_aids'][1]['arc_crossing_selected_ms']=19
        self.meta['walking_energy']['records'][1]['guarded_ms']=19
        self.read()
        same=observed_prediction(self.folder)['events'][1]
        self.assertTrue(same['forecast_before_transfer'] and not same['width_extended'] and not same['executed_extension'])
        self.meta['walking_energy']['arc_aids'][1].pop('arc_approach_request_j')
        self.read()
        self.assertFalse(observed_prediction(self.folder)['archive_complete'])
        self.meta['walking_profile']='left_return_history_v1'
        self.read()
        self.assertIsNone(observed_prediction(self.folder))

    def test_true_zero_and_veto(self):
        self.meta['walking_energy']['records'][1]['executed'] = False
        result = self.read()
        self.assertEqual((result['evaluated'],result['matching_donors'],result['width_increased'],
                          result['executed_increased']), (2,2,2,0))
        self.assertIsNone(result['first_executed_change'])
        self.meta['walking_energy'].update(arc_aids=[],records=[],total=0,stored=0)
        self.assertEqual(self.read()['evaluated'],0)


if __name__ == '__main__':
    unittest.main()
