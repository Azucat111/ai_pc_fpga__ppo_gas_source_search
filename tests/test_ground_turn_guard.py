"""Reject bad turn direction, missing feedback, excessive angle, or translation."""
import copy
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from check_ground_turn import check_turn_feedback


class GroundTurnGuardTests(unittest.TestCase):
    def setUp(self):
        self.config=dict(wheel_diameter_left_m=.067,wheel_diameter_right_m=.067,
                         counts_per_revolution_left=1320,counts_per_revolution_right=1320)
        self.base=dict(boot_id=123,enabled=True,imu_yaw=1.5,imu_age_ms=18,
                       geometry_valid=True,imu_frame_confirmed=True,
                       encoder_left_count=0,encoder_right_count=0,
                       wheel_rpm_left=0,wheel_rpm_right=0,
                       travelled_distance_m=1,distance_m=0,
                       errors=dict(encoder_m1=0,encoder_m2=0))

    def test_both_turn_directions_with_correct_feedback(self):
        for sign in (-1,1):
            row=dict(self.base,encoder_left_count=-sign*100,encoder_right_count=sign*100,
                     wheel_rpm_left=-sign*12,wheel_rpm_right=sign*12,travelled_distance_m=1.04)
            check_turn_feedback(row,self.base,math.radians(sign*25),sign*30,3,self.config)

    def test_invalid_imu_or_session_stops(self):
        for change in (dict(imu_yaw=None),dict(imu_age_ms=151),dict(boot_id=124),
                       dict(enabled=False),dict(imu_frame_confirmed=False)):
            with self.subTest(change=change),self.assertRaises(RuntimeError):
                check_turn_feedback(dict(self.base,**change),self.base,0,30,0,self.config)

    def test_wrong_wheel_direction_speed_stall_and_encoder_error_stop(self):
        cases=[(dict(self.base,encoder_left_count=9),0),
               (dict(self.base,wheel_rpm_right=61),0),(dict(self.base),2.6)]
        row=copy.deepcopy(self.base);row['errors']['encoder_m1']=4;cases.append((row,0))
        for row,elapsed in cases:
            with self.subTest(row=row,elapsed=elapsed),self.assertRaises(RuntimeError):
                check_turn_feedback(row,self.base,0,30,elapsed,self.config)

    def test_wrong_yaw_excessive_angle_or_travel_stops(self):
        for change,angle in (({},-9),({},61),(dict(travelled_distance_m=1.251),0),
                             (dict(distance_m=.121),0)):
            with self.subTest(change=change,angle=angle),self.assertRaises(RuntimeError):
                check_turn_feedback(dict(self.base,**change),self.base,math.radians(angle),30,0,self.config)

    def test_opposite_wheel_travel_cannot_cancel_displacement_guard(self):
        row=dict(self.base,encoder_left_count=-1600,encoder_right_count=1600)
        # Centre odometry is zero while each wheel has moved more than 0.25m.
        with self.assertRaisesRegex(RuntimeError,'individual wheel'):
            check_turn_feedback(row,self.base,math.radians(20),30,3,self.config)

    def test_gyro_prevents_wrong_direction_or_large_unseen_body_turn(self):
        for gyro in (-9,106):
            with self.subTest(gyro=gyro),self.assertRaisesRegex(RuntimeError,'Gyro'):
                check_turn_feedback(self.base,self.base,math.radians(20),90,0,self.config,math.radians(gyro))
        check_turn_feedback(self.base,self.base,math.radians(89),90,0,self.config,math.radians(91))

    def test_gyro_core_uses_body_gyro_while_fused_yaw_is_diagnostic(self):
        base=dict(self.base,turn_gyro_heading=True)
        check_turn_feedback(base,base,math.radians(-20),90,0,self.config,math.radians(40))
        check_turn_feedback(base,base,math.radians(130),90,0,self.config,math.radians(90))
        with self.assertRaisesRegex(RuntimeError,'Gyro'):
            check_turn_feedback(base,base,math.radians(90),90,0,self.config,math.radians(106))
