"""Service stop hooks clear permission without enabling or resetting the loaded core."""
import unittest
from unittest.mock import Mock,patch
from car.emergency_stop import disable_motion


class EmergencyStopTests(unittest.TestCase):
    def test_only_permission_clear_is_written(self):
        mmio=Mock();mmio.read.return_value=0x43415231
        zero=dict(motor_permission=False,drive_active=False,duty_permille=dict(left=0,right=0),output_logic=dict(A=False,B=False))
        with patch('car.emergency_stop.read_diagnostics',return_value=zero):self.assertIs(disable_motion(mmio),zero)
        mmio.write.assert_called_once_with(0x14,0)

    def test_unknown_overlay_is_never_modified(self):
        mmio=Mock();mmio.read.return_value=0
        with self.assertRaises(RuntimeError):disable_motion(mmio)
        mmio.write.assert_not_called()

    def test_failure_to_reach_zero_is_not_reported_as_success(self):
        mmio=Mock();mmio.read.return_value=0x43415231
        live=dict(motor_permission=True,drive_active=True,duty_permille=dict(left=100,right=100),output_logic=dict(A=True,B=False))
        with patch('car.emergency_stop.read_diagnostics',return_value=live),patch('car.emergency_stop.time.monotonic',side_effect=[0,.3]):
            with self.assertRaises(RuntimeError):disable_motion(mmio)
        mmio.write.assert_called_once_with(0x14,0)
