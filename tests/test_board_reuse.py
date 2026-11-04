"""Enabled commissioning service can reuse the disabled, identified FPGA core."""
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

from car import board
from car.config import load_config
from pathlib import Path


class BoardReuseTests(unittest.TestCase):
    def run_case(self,enabled=False,signature=0x494d5532,gyro_capability=0x47595231,turn_capability=0x47595232):
        c=load_config(Path(__file__).resolve().parents[1]/'config/car.json')
        c.update(imu_mount_confirmed=True,wiring_confirmed=True)
        values={0:0x43415231,4:0x10000,0x14:int(enabled),0xcc:signature,0xd0:gyro_capability,0xd4:turn_capability}
        io=Mock();io.read.side_effect=lambda a:values.get(a,0)
        io.write.side_effect=lambda a,v:values.__setitem__(a,v)
        overlay=Mock(side_effect=AssertionError('Must not reload FPGA'))
        fake=SimpleNamespace(Overlay=overlay,MMIO=lambda *_:io,Clocks=SimpleNamespace(fclk0_mhz=5))
        now=[0.0]
        with patch.dict(sys.modules,{'pynq':fake}), \
             patch.object(sys,'argv',['car-board','--reuse-overlay','--enable','--no-bme','--seconds','.1']), \
             patch.object(board,'load_config',return_value=c),patch.object(board,'PLBus') as bus, \
             patch.object(board.time,'monotonic',side_effect=lambda:now[0]), \
             patch.object(board.time,'sleep',side_effect=lambda t:now.__setitem__(0,now[0]+t)), \
             patch('builtins.print'):
            bus.return_value.transfer.return_value=b'\x01\0\0'
            if enabled or signature!=0x494d5532 or gyro_capability!=0x47595231 or turn_capability!=0x47595232:
                with self.assertRaises(RuntimeError):board.main()
                io.write.assert_not_called()
                bus.assert_not_called()
            else:
                board.main()
                self.assertEqual(values[0x14],0)
                self.assertTrue(any(call.args==(0x14,1) for call in io.write.call_args_list))
            overlay.assert_not_called()

    def test_reuse_and_finite_exit_clear_permission(self):self.run_case()
    def test_reject_already_enabled_core(self):self.run_case(enabled=True)
    def test_reject_heading_without_last_good_capability(self):self.run_case(signature=0)
    def test_reject_heading_without_plausibility_capability(self):self.run_case(signature=0x494d5531)
    def test_reject_old_core_without_gyro_straight_control(self):self.run_case(gyro_capability=0)

    def test_reject_old_core_without_gyro_turn_control(self):self.run_case(turn_capability=0)
