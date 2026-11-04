"""Reject the recorded wrong-sign yaw feedback while allowing valid heading motion."""
import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from check_ground_straight import HeadingConsistency


def row(t,yaw,gyro):return dict(timestamp_ms=t,imu_yaw=yaw,imu_gyro_z=gyro)


class HeadingConsistencyTests(unittest.TestCase):
    def test_new_gyro_control_records_magnetic_drift_without_steering_from_it(self):
        h=HeadingConsistency(dict(row(0,0,0),straight_gyro_heading=True))
        for i in range(1,301):h.update(row(i*20,i*.002,0))
        self.assertEqual(h.gyro_integral,0)
        self.assertGreater(h.yaw_change,math.radians(15))

    def test_new_gyro_control_stops_actual_body_heading_excursion(self):
        h=HeadingConsistency(dict(row(0,0,1),straight_gyro_heading=True))
        with self.assertRaisesRegex(RuntimeError,'Gyro-relative'):
            for i in range(1,100):h.update(row(i*20,0,1))

    def test_yaw_wrap_and_clock_wrap_are_small_steps(self):
        h=HeadingConsistency(row(0xfffffff0,3.13,1))
        r=row(4,-3.13,1);h.update(r)
        self.assertAlmostEqual(h.gyro_integral,.02)
        self.assertLess(abs(r['heading_yaw_gyro_disagreement_deg']),1)

    def test_valid_clockwise_and_counterclockwise_motion(self):
        for sign in (-1,1):
            h=HeadingConsistency(row(0,0,sign*.1))
            for i in range(1,301):h.update(row(i*20,sign*.1*i*.02,sign*.1))
            self.assertAlmostEqual(h.yaw_change,h.gyro_integral)

    def test_gap_or_missing_feedback_aborts(self):
        for r in (row(501,0,0),row(20,None,0),row(20,0,None)):
            with self.subTest(r=r),self.assertRaises(RuntimeError):HeadingConsistency(row(0,0,0)).update(r)

    def test_real_previous_short_straight_is_accepted(self):
        data=json.loads((Path(__file__).parent/'fixtures/straight_heading_agreement_20261026.json').read_text())
        records=[dict(zip(data['fields'],x)) for x in data['samples']]
        h=HeadingConsistency(records[0])
        for r in records[1:]:h.update(r)
        self.assertLess(abs(h.yaw_change-h.gyro_integral),math.radians(6))

    def test_real_right_veering_trace_aborts_before_full_six_seconds(self):
        data=json.loads((Path(__file__).parent/'fixtures/straight_heading_disagreement_20261027.json').read_text())
        records=[dict(zip(data['fields'],x)) for x in data['samples']]
        h=HeadingConsistency(records[0]);failed_at=None
        for r in records[1:]:
            try:h.update(r)
            except RuntimeError as e:
                self.assertIn('disagreed',str(e));failed_at=r['timestamp_ms'];break
        self.assertIsNotNone(failed_at)
        self.assertLess((failed_at-records[0]['timestamp_ms'])&0xffffffff,5000)
