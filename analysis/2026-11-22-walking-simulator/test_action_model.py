import math
import unittest
from types import SimpleNamespace

import numpy as np

from action_model import LIMIT, Pulse, Recording, State, advance, compare, first_return_status, observed
from phase_dynamics import PhaseState, coast_endpoint, crossing_speed, energy_features, training_windows, zero_followup


class TestActionRollout(unittest.TestCase):
    def setUp(self):
        self.state = State(angle=3., speed=20., acceleration=0., foot_difference=4., peak_amplitude=6.)

    def test_current_alignment_rise_and_tail(self):
        p = Pulse(0,2,22,-1,0.,300.,.03,True,0,20)
        self.assertAlmostEqual(p.current(2),0.)
        self.assertAlmostEqual(p.current(22),300.*(1-math.exp(-.02/.03)))
        self.assertAlmostEqual(p.current(92),p.current(22)/math.e)

    def test_future_pulse_does_not_affect_an_earlier_crossing(self):
        class CurrentDriven:
            def acceleration(self,s,previous,current):
                return current
        a = advance(CurrentDriven(),self.state,0,lambda t:0.,0.,2.8)
        b = advance(CurrentDriven(),self.state,0,lambda t:500. if t>=15 else 0.,0.,2.8)
        self.assertEqual(a,b)
        self.assertEqual(a['time_ms'],10.)
        self.assertEqual(a['outcome'],'section')

    def test_reversal_terminates_before_zero(self):
        class Deceleration:
            def acceleration(self,s,previous,current):
                return -200.
        r = advance(Deceleration(),self.state,0,lambda t:0.,0.,0.)
        self.assertEqual(r['outcome'],'reversed')
        self.assertEqual(r['angle_deg'],2.)
        self.assertEqual(r['time_ms'],100.)
        self.assertLess(r['time_ms'],LIMIT)

    def test_main_reserve_uses_only_known_completed_pulses(self):
        rec = Recording.__new__(Recording)
        def event(ending,width,peak,aid,assisted):
            return dict(output_end_ms=ending,actual_elapsed_ms=width,peak_ms=peak,side=-1,
                        completion_reason=1,completion_valid=True,return_event=aid,assisted_cycle=assisted)
        rec.records = [event(100,60,2,False,False),event(300,48,3,False,True),
                       event(350,20,4,True,True),event(500,70,4,False,True)]
        self.assertEqual(rec.reserve(350,4),(48,60))
        self.assertEqual(rec.reserve(30,4),(80,80))

    @staticmethod
    def trace(times):
        t = np.asarray(times)
        return SimpleNamespace(t=t,x=4.-.02*t,v=np.full(len(t),20.),pulses=[])

    def test_observation_refuses_gap_across_horizon(self):
        rec = self.trace([*range(0,141,5),170])
        self.assertEqual(observed(rec,0,.1),{'outcome':'censored','reason':'imu_gap'})

    def test_observation_excludes_crossing_before_requested_time(self):
        rec = self.trace(range(0,171,5))
        self.assertEqual(observed(rec,4,3.96),{'outcome':'censored','reason':'section_already_passed'})
        self.assertEqual(observed(rec,4,3.9)['time_ms'],1.)

    def test_observation_interpolates_section_inside_last_partial_slot(self):
        rec = self.trace(range(0,171,7))
        self.assertEqual(observed(rec,1,1.)['time_ms'],149.)
        self.assertEqual(observed(rec,1,.8)['time_ms'],LIMIT)

    def test_second_inward_swing_excluded_only_after_reversal_is_observable(self):
        r = SimpleNamespace(t=[0,5,10,15,20],x=[3.,2.6,2.61,2.85,2.71],v=[1.,1.,-1.,-1.,1.])
        self.assertEqual(first_return_status(r,0,10),'ok')
        self.assertEqual(first_return_status(r,0,20),'outside_first_inward_return')

    def test_pending_and_active_pulses_hold_new_comparison(self):
        r = SimpleNamespace(run=1,pulses=[Pulse(10,13,30,-1,0.,330.,.03,True,0,17)],peaks=[])
        self.assertEqual(compare(r,10,None,[],[],[])['status'],'outside_positive_return')
        for t in (11,13,29):
            self.assertEqual(compare(r,t,None,[],[],[])['status'],'previous_command_pending_or_active')
        self.assertEqual(compare(r,30,None,[],[],[])['status'],'outside_positive_return')

    def test_partial_training_window_ends_at_zero(self):
        t=np.arange(0,56,5)
        peak={'peak_telemetry_capture_time_ms':1}
        r=SimpleNamespace(t=t,x=(46-t)*.02,v=20.+t*.1,segments=[(peak,0,10,'crossed')])
        r.current=lambda time:0.
        r.observation=lambda time,p:State((46-time)*.02,20.+time*.1,100.,0.,3.)
        rows=training_windows(r)
        self.assertEqual(len(rows),1)
        self.assertTrue(rows[0][-1])
        self.assertAlmostEqual(rows[0][-2],100.)

    def test_rollout_freezes_observed_feet_and_tracks_peak_kinetic_energy(self):
        s=PhaseState(2.,15.,-100.,2.,6.,150.,20.,3.,-10.,5.,6.)
        n=s.after_slot(1.6,10.,-200.)
        self.assertEqual((n.left_angle,n.left_rate,n.right_rate),(3.,-10.,5.))
        self.assertEqual((n.elapsed_ms,n.fastest_inward_dps),(175.,20.))
        self.assertAlmostEqual(energy_features(n,0.,0.)[-3],
                               .5*(math.radians(20)**2-math.radians(10)**2))

    def test_zero_pulse_and_later_intervention_are_distinguished(self):
        cutoff={'outcome':'censored'}
        o=SimpleNamespace(state=SimpleNamespace(time_ms=200,peak_ms=100), end_ms=260, zero_speed=16.,
                          future_pulse=False, outcome='crossed')
        r=SimpleNamespace(t=np.array([255,260,265]),v=np.array([16.,18.,20.]),x=np.array([.1,0.,-.1]),
                          bias_difference=np.array([1.,2.,9.]),gyro_scale=.8,left_zeros={100:{
                              'zero_cross_time_ms':260,'interpolated_crossing_time_ms':260.,'zero_cross_rate_dps':-16.}},
                          pulses=[SimpleNamespace(decision=260,start=260)])
        # At ZERO the event's -16 dps is on the scaled MEKF series. An output
        # starting at 260 excludes the simultaneous stored 2 dps bias offset.
        self.assertEqual(coast_endpoint(r,o,cutoff)['speed_dps'],19.)
        self.assertEqual(coast_endpoint(r,o,cutoff)['speed_source'],'synchronized_gyro_conversion')
        r.peaks=[dict(peak_time_ms=100)]
        self.assertEqual(zero_followup(r,200,100,cutoff),coast_endpoint(r,o,cutoff))
        r.bias_difference[:]=math.nan
        self.assertIsNone(crossing_speed(r,200,260,100))
        r.pulses=[]
        self.assertEqual(crossing_speed(r,200,260,100),dict(speed_dps=18.,speed_source='pre_command_gyro_bracket'))
        o.future_pulse=True
        self.assertEqual(coast_endpoint(r,o,cutoff),cutoff)
        o.future_pulse=False
        o.end_ms=400
        self.assertEqual(coast_endpoint(r,o,cutoff),cutoff)

    def test_negative_requested_endpoint_never_extrapolates_through_zero(self):
        r=self.trace(range(0,401,5))
        r.run=1
        r.peaks=[dict(peak_time_ms=0, peak_telemetry_capture_time_ms=0, physical_peak_side=1)]
        r.aids=[]
        r.observation=lambda t,p: State(2.,20.,0.,4.,6.)
        r.reserve=lambda t,p:(30,80)
        r.energy={'omega2_j':.1,'rate_min_dps':0.,'rate_max_dps':100.}
        model=SimpleNamespace(acceleration=lambda s,old,new: 0.)
        donor=Pulse(0,2,22,-1,0.,300.,.03,True,0,20)
        result=compare(r,100,model,[(2,self.state,donor)],(0,),(0,),-1.91)
        self.assertEqual(result['requested_section_deg'],-1.91)
        self.assertEqual(result['target_section_deg'],0.)
        self.assertFalse(result['receiving_side_prediction_available'])
        self.assertEqual(result['scenarios'][0]['angle_deg'],0.)


if __name__=='__main__':
    unittest.main()
