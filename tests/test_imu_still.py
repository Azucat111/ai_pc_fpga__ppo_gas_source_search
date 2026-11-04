"""Stationary diagnostics must remain STOP-only and detect invalid observations."""
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import check_imu_still as still


def sample():
    return dict(boot_id=123,last_action='stop',imu_yaw=3.13,imu_age_ms=17,imu_gyro_z=0,
                wheel_rpm_left=0,wheel_rpm_right=0,encoder_left_count=0,encoder_right_count=0,
                errors=dict(imu=0,adc=0,uart=0,encoder_m1=0,encoder_m2=0))


class FakeClient:
    def __init__(self,change=None):self.clock=0;self.modes=[];self.change=change;self.count=0
    def state(self):
        row=sample();row['imu_yaw']=(3.13+self.count*.001+math.pi)%(2*math.pi)-math.pi
        return row
    def receive(self,**kwargs):
        self.clock+=.02;self.count+=1;row=self.state()
        if self.change and self.count>5:row.update(self.change)
        return row
    def send(self,mode):self.modes.append(mode);return self.state()


class ImuStillTests(unittest.TestCase):
    def test_only_stop_commands_and_yaw_wrap_is_preserved(self):
        client=FakeClient()
        with patch.object(still.time,'monotonic',side_effect=lambda:client.clock):
            summary,records=still.observe(client)
        self.assertTrue(summary['ok']);self.assertGreater(len(records),500)
        self.assertEqual(client.modes,[0,0])
        self.assertAlmostEqual(summary['yaw_drift_deg'],math.degrees((client.count-1)*.001),places=6)
        self.assertLess(summary['yaw_range_deg'],40)

    def test_boot_motion_feedback_or_imu_invalidity_aborts(self):
        base=sample()
        for change in (dict(boot_id=124),dict(last_action='turn_relative'),dict(imu_yaw=None),
                       dict(imu_age_ms=151),dict(encoder_left_count=4),dict(wheel_rpm_right=1.1)):
            with self.subTest(change=change),self.assertRaises(RuntimeError):
                still.check_stationary(dict(base,**change),base)

    def test_observation_failure_still_attempts_stop(self):
        client=FakeClient(dict(encoder_left_count=4))
        with patch.object(still.time,'monotonic',side_effect=lambda:client.clock):
            summary,records=still.observe(client)
        self.assertFalse(summary['ok']);self.assertIn('Wheel movement',summary['error'])
        self.assertTrue(summary['stop_acknowledged']);self.assertEqual(client.modes,[0,0])
