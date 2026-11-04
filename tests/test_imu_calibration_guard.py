"""Use physical level as reference, reject movement, and check residual gyro bias."""
import copy
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from calibrate_imu import check_setup,result_is_usable


class CalibrationGuardTests(unittest.TestCase):
    def setUp(self):
        self.rows=[dict(imu_yaw=1.5,imu_gyro_z=-.014,imu_age_ms=18,enabled=False,
                        encoder_left_count=0,encoder_right_count=0) for _ in range(45)]
        self.acc=[(0,0,8192)]*5

    def test_physical_level_reference_allows_uncalibrated_accelerometer_offset(self):
        check_setup(self.rows,self.acc)
        tilt=math.radians(4.3)
        tilted=[(round(8192*math.sin(tilt)),0,round(8192*math.cos(tilt)))]*5
        result=check_setup(self.rows,tilted)
        self.assertTrue(result['apparent_tilt_warning'])
        self.assertAlmostEqual(result['accel_tilt_median_deg'],4.3,places=1)
        with self.assertRaisesRegex(RuntimeError,'face up'):
            check_setup(self.rows,[(0,0,-8192)]*5)

    def test_movement_invalid_data_or_missing_acceleration_stops_preflight(self):
        for change in (dict(imu_yaw=None),dict(imu_age_ms=151),dict(enabled=True),
                       dict(encoder_left_count=1)):
            rows=copy.deepcopy(self.rows);rows[-1].update(change)
            with self.subTest(change=change),self.assertRaises(RuntimeError):
                check_setup(rows,self.acc)
        rows=copy.deepcopy(self.rows)
        for i,r in enumerate(rows):r['imu_gyro_z']=.02 if i%2 else -.02
        with self.assertRaisesRegex(RuntimeError,'varies'):check_setup(rows,self.acc)
        with self.assertRaisesRegex(RuntimeError,'nonzero'):check_setup(self.rows,[(0,0,0)]*5)

    def test_success_status_alone_does_not_pass_large_bias_or_dropouts(self):
        s=dict(samples=230,valid=230,max_age_ms=38,gyro_mean=.0002,gyro_std=.0008)
        self.assertTrue(result_is_usable(s))
        for change in (dict(gyro_mean=-.014),dict(gyro_std=.01),dict(valid=229),dict(max_age_ms=151)):
            with self.subTest(change=change):self.assertFalse(result_is_usable(dict(s,**change)))
