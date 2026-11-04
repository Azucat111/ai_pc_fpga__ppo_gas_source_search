"""First ground trial must stop on lost heading, excessive travel, or bad feedback."""
import copy
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from check_ground_straight import check_ground_feedback


class GroundGuardTests(unittest.TestCase):
    def setUp(self):
        self.base = dict(boot_id=123, enabled=True, encoder_left_count=0, encoder_right_count=0,
                         wheel_rpm_left=20, wheel_rpm_right=20,
                         errors=dict(encoder_m1=0, encoder_m2=0), imu_yaw=3.13, imu_age_ms=18,
                         geometry_valid=True, imu_frame_confirmed=True, travelled_distance_m=1)

    def test_crossing_pi_is_small_motion(self):
        r = dict(self.base, imu_yaw=-3.13, encoder_left_count=100, encoder_right_count=100,
                 travelled_distance_m=1.2)
        check_ground_feedback(r, self.base, 3, 20)

    def test_imu_dropout_staleness_and_lost_confirmation_stop(self):
        for change in (dict(imu_yaw=None), dict(imu_age_ms=151),
                       dict(geometry_valid=False), dict(imu_frame_confirmed=False)):
            with self.subTest(change=change), self.assertRaisesRegex(RuntimeError, 'fresh IMU'):
                check_ground_feedback(dict(self.base, **change), self.base, 0, 20)

    def test_heading_and_distance_bounds_stop(self):
        for change, message in ((dict(imu_yaw=3.13+math.radians(16)), 'Heading'),
                                (dict(travelled_distance_m=1.501), 'travel bound'),
                                (dict(travelled_distance_m=None), 'travel bound')):
            with self.subTest(change=change), self.assertRaisesRegex(RuntimeError, message):
                check_ground_feedback(dict(self.base, **change), self.base, 0, 20)

    def test_stall_reverse_overspeed_and_invalid_encoder_stop(self):
        cases = [(dict(self.base), 2.1)]
        for change in (dict(encoder_left_count=-9), dict(wheel_rpm_right=61), dict(boot_id=124)):
            cases.append((dict(self.base, **change), 0))
        r = copy.deepcopy(self.base);r['errors']['encoder_m2'] = 4;cases.append((r, 0))
        for row, elapsed in cases:
            with self.subTest(row=row, elapsed=elapsed), self.assertRaises(RuntimeError):
                check_ground_feedback(row, self.base, elapsed, 20)
