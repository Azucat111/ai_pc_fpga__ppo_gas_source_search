"""Motor-connected pulses must not arm an older, unwoken bitstream."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


class WakeGuardTest(unittest.TestCase):
    def test_motor_pulse_rejects_older_bitstream_before_enabling(self):
        spec = importlib.util.spec_from_file_location('arm_probe_guard', ROOT / 'tools/arm_bare_car.py')
        tool = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(tool)
        class IO:
            def __init__(self):
                self.writes = []
            def read(self, address):
                return {0: 0x43415231, 0x64: 0x44424731, 0x68: 5000000}.get(address, 0)
            def write(self, address, value):
                self.writes.append((address, value))
        io = IO()
        fake_pynq = SimpleNamespace(MMIO=lambda *args: io, Clocks=SimpleNamespace(fclk0_mhz=5.0))
        argv = ['arm_bare_car.py', '--motor-pulse', '--wheels-raised', '--seconds', '30']
        with patch.dict(sys.modules, {'pynq': fake_pynq}), patch.object(sys, 'argv', argv), \
                patch.object(tool.Path, 'glob', return_value=[]):
            with self.assertRaisesRegex(RuntimeError, 'wake-capable'):
                tool.main()
        self.assertEqual(io.writes, [])


if __name__ == '__main__':
    unittest.main()
