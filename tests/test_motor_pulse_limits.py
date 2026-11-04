"""Reject unsupported motor trial requests before serial or board access."""
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]


class MotorPulseLimits(unittest.TestCase):
    def test_invalid_requests_never_reach_hardware(self):
        common = ['--motor-pulse', '--wheels-raised']
        cases = [
            ('check_car_uart.py', ['--port', 'UNUSED'] + common + ['--seconds', '5']),
            ('check_car_uart.py', ['--port', 'UNUSED'] + common + ['--pulse-duty-permille', '700', '--seconds', '5.1']),
            ('check_car_uart.py', ['--port', 'UNUSED', '--motor-pulse', '--pulse-duty-permille', '700', '--seconds', '5']),
            ('arm_bare_car.py', common + ['--pulse-duty-permille', '701']),
            ('arm_bare_car.py', ['--bare-board', '--pulse-duty-permille', '700']),
        ]
        for tool, args in cases:
            with self.subTest(tool=tool, args=args):
                result = subprocess.run([sys.executable, str(ROOT / 'tools' / tool)] + args,
                                        capture_output=True, text=True)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertNotIn('Traceback', result.stderr)
