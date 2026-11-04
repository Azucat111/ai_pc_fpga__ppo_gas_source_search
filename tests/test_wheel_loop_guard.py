"""Abort a real trial when feedback is missing, reversed, unsafe, or from another session."""
import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from check_wheel_loop import check_feedback


class WheelLoopGuardTests(unittest.TestCase):
    def setUp(self):
        self.base = dict(boot_id=123, enabled=True, encoder_left_count=100,
                         encoder_right_count=200, wheel_rpm_left=0, wheel_rpm_right=0,
                         errors=dict(encoder_m1=0, encoder_m2=0))

    def test_valid_forward_and_reverse_feedback(self):
        for rpm, delta in ((20, 20), (-20, -20)):
            r = dict(self.base, encoder_right_count=200 + delta, wheel_rpm_right=rpm)
            check_feedback(r, self.base, 'right', rpm, 3)

    def test_missing_feedback_aborts_after_startup_allowance(self):
        check_feedback(self.base, self.base, 'right', 20, 1.9)
        with self.assertRaisesRegex(RuntimeError, 'No encoder'):
            check_feedback(self.base, self.base, 'right', 20, 2.1)

    def test_wrong_direction_aborts(self):
        for rpm, delta in ((20, -9), (-20, 9)):
            with self.assertRaisesRegex(RuntimeError, 'opposite'):
                check_feedback(dict(self.base, encoder_right_count=200 + delta), self.base, 'right', rpm, .1)

    def test_other_wheel_and_overspeed_abort(self):
        for change, message in ((dict(encoder_left_count=109), 'Unselected'),
                                (dict(wheel_rpm_right=61), 'speed guard')):
            with self.assertRaisesRegex(RuntimeError, message):
                check_feedback(dict(self.base, **change), self.base, 'right', 20, .1)

    def test_session_change_and_permission_loss_abort(self):
        for change in (dict(boot_id=124), dict(enabled=False)):
            with self.assertRaisesRegex(RuntimeError, 'session changed'):
                check_feedback(dict(self.base, **change), self.base, 'right', 20, .1)

    def test_invalid_transitions_abort(self):
        r = copy.deepcopy(self.base)
        r['errors']['encoder_m1'] = 4
        with self.assertRaisesRegex(RuntimeError, 'invalid transitions'):
            check_feedback(r, self.base, 'right', 20, .1)

    def test_both_wheels_need_correct_feedback(self):
        r = dict(self.base, encoder_left_count=120, encoder_right_count=220,
                 wheel_rpm_left=20, wheel_rpm_right=20)
        check_feedback(r, self.base, 'both', 20, 3)
        for side in ('left', 'right'):
            bad = dict(r)
            bad['encoder_' + side + '_count'] = self.base['encoder_' + side + '_count'] - 9
            with self.assertRaisesRegex(RuntimeError, 'opposite'):
                check_feedback(bad, self.base, 'both', 20, .1)
