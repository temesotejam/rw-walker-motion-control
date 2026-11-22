import unittest
from types import SimpleNamespace

import numpy as np

from action_model import Pulse, compare
from loss_observer import coast_forecast, innovation, persistent_loss
from phase_dynamics import PhaseState


class LossObserverTest(unittest.TestCase):
    def setUp(self):
        t = np.arange(0, 260, 5)
        self.peak = {'peak_time_ms':0, 'physical_peak_side':1, 'peak_telemetry_capture_time_ms':0}
        self.rec = SimpleNamespace(run=1,t=t,x=4.3-.02*t,v=30.-t*.1,pulses=[],aids=[],peaks=[self.peak],
                                   reserve=lambda time,p:(30,80),
                                   energy={'omega2_j':.1,'rate_min_dps':0.,'rate_max_dps':100.})
        self.rec.observation = lambda time,p: PhaseState(4.3-.02*time,30.-.1*time,-100.,0.,5.,
                                                         time,30.,0.,0.,0.,5.)
        self.fit = SimpleNamespace(acceleration=lambda s,old,new:0.)
        self.example = (2,self.rec.observation(75,self.peak),Pulse(0,0,20,-1,0.,300.,.03,True,0,20))

    def test_observer_consumes_only_earlier_slots_and_fades_without_new_observations(self):
        def past_current(time):
            self.assertLess(time,75)
            return 0.
        fit, audit = coast_forecast(self.fit,self.rec,self.peak,77,past_current)
        self.assertEqual((audit['previous']['from_ms'],audit['latest']['to_ms']),(25,75))
        self.assertEqual(audit['correction_dps2'],-100.)
        self.assertLess(audit['latest']['energy_error_per_inertia_s2'],0)
        self.assertAlmostEqual(fit.acceleration(self.rec.observation(77,self.peak),0,0),-100.)
        self.assertAlmostEqual(fit.acceleration(self.rec.observation(102,self.peak),0,0),-50.)
        delayed = dict(self.peak,peak_telemetry_capture_time_ms=51)
        self.assertIsNone(innovation(self.rec,delayed,77,self.fit,past_current))
        self.rec.x += 1.
        fit, audit = coast_forecast(self.fit,self.rec,self.peak,77,past_current)
        self.assertIs(fit,self.fit)
        self.assertFalse(audit['in_validated_angle_range'])

    def test_loss_requires_two_deficits_and_current_deceleration(self):
        def slot(error,change):
            return dict(acceleration_error_dps2=error,measured_speed_change_dps=change)
        self.assertEqual(persistent_loss(slot(-80,-2),slot(-100,-3)),-80)
        for prior, latest in [(None,slot(-100,-2)),(slot(12,1),slot(-100,-2)),
                              (slot(-80,-2),slot(-70,1)),(slot(-80,-2),None)]:
            self.assertEqual(persistent_loss(prior,latest),0.)

    def test_width_comparison_is_paired_and_future_command_cannot_change_coast_forecast(self):
        def forecast(widths):
            return compare(self.rec,75,self.fit,[self.example],widths,(0,),0.)
        base = forecast((0,1,20))
        self.fit.coast_observer = coast_forecast
        monitored = forecast((0,1,20))
        self.assertEqual(monitored['scenarios'],base['scenarios'])
        self.assertLess(monitored['coast_forecast']['speed_dps'],base['scenarios'][0]['speed_dps'])
        self.assertEqual(monitored['coast_forecast'],forecast((0,))['coast_forecast'])
        self.assertNotIn('coast_forecast',forecast((1,20)))
        self.rec.pulses.append(Pulse(90,91,111,-1,0.,300.,.03,True,0,20))
        self.rec.v[self.rec.t>75] = 100.
        replay = forecast((0,1,20))
        self.assertEqual(replay['scenarios'],monitored['scenarios'])
        self.assertEqual(replay['coast_forecast'],monitored['coast_forecast'])
        self.assertEqual(replay['coast_observation'],monitored['coast_observation'])


if __name__ == '__main__':
    unittest.main()
