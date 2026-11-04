"""Sensor bring-up with motor VM on must reuse the disabled overlay without loading it."""
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

from car import diagnose


class DiagnoseReuseTests(unittest.TestCase):
    def run_reuse(self, enabled=False):
        io=Mock()
        io.read.side_effect=lambda a:{0:0x43415231,4:0x10000,0x14:int(enabled)}.get(a,0)
        overlay=Mock(side_effect=AssertionError('FPGA reload forbidden'))
        pynq=SimpleNamespace(Overlay=overlay,MMIO=lambda *_:io,Clocks=SimpleNamespace(fclk0_mhz=5),__version__='test')
        now=[0.0]
        with patch.dict(sys.modules,{'pynq':pynq}), \
             patch.object(sys,'argv',['car-diagnose','--reuse-overlay','--no-bme','--seconds','.1']), \
             patch.object(diagnose,'configure') as configure, \
             patch.object(diagnose,'PLBus') as bus, \
             patch.object(diagnose,'snapshot',return_value=(1,[0]*45)), \
             patch.object(diagnose.time,'monotonic',side_effect=lambda:now[0]), \
             patch.object(diagnose.time,'sleep',side_effect=lambda t:now.__setitem__(0,now[0]+t)), \
             patch('builtins.print'):
            bus.return_value.transfer.return_value=b'\x01\0\0'
            if enabled:
                with self.assertRaisesRegex(RuntimeError,'Motor permission'):
                    diagnose.main()
                bus.assert_not_called()
                io.write.assert_not_called()
            else:
                diagnose.main()
                self.assertIn(call(0x14,0),io.write.call_args_list)
            overlay.assert_not_called()
            configure.assert_not_called()
        return io

    def test_disabled_overlay_reused_without_configuration_or_reload(self):
        self.run_reuse()

    def test_enabled_core_rejected_before_sensor_access(self):
        self.run_reuse(enabled=True)
