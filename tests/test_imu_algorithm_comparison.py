"""A/B diagnostics must never enable motion or alter calibration registers."""
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import compare_imu_algorithms as ab
from test_imu_still import sample


class MMIO:
    def __init__(self,permit=0):self.permit=permit;self.writes=[]
    def read(self,address):return self.permit
    def write(self,address,value):
        assert address==0x14 and value==0
        self.writes.append((address,value));self.permit=value


class Bus:
    def __init__(self):self.mode=9;self.writes=[]
    def transfer(self,address,register,write=b'',read=0):
        assert address==0x23 and register==0x61
        if write:self.mode=write[0];self.writes.append(self.mode)
        return bytes([self.mode]) if read else b''


class AlgorithmComparisonTests(unittest.TestCase):
    def test_algorithm_readback_waits_for_device_to_apply_setting(self):
        class DelayedBus(Bus):
            def transfer(self,address,register,write=b'',read=0):
                if write:self.pending=write[0];self.remaining=3;self.writes.append(write[0]);return b''
                self.remaining-=1
                if self.remaining==0:self.mode=self.pending
                return bytes([self.mode])
        bus=DelayedBus()
        with patch.object(ab.time,'sleep') as pause:ab.select_algorithm(MMIO(),bus,6)
        self.assertEqual(bus.mode,6);self.assertEqual(pause.call_count,2)

    def test_successful_9_6_9_preserves_calibration_and_disabled_gate(self):
        mmio=MMIO();bus=Bus();rows=[sample() for _ in range(100)]
        with patch.object(ab,'collect',return_value=rows),patch('builtins.print'):
            result=ab.compare(mmio,bus,lambda:None)
        self.assertTrue(result['ok']);self.assertEqual(bus.writes,[6,9])
        self.assertEqual(result['restored_algorithm'],9);self.assertFalse(result['motor_permission'])

    def test_failure_in_alternative_restores_original_without_enabling(self):
        mmio=MMIO();bus=Bus();rows=[sample() for _ in range(100)]
        def collect(*_):
            if bus.mode==6:raise RuntimeError('injected read failure')
            return rows
        with patch.object(ab,'collect',side_effect=collect),patch('builtins.print'):
            result=ab.compare(mmio,bus,lambda:None)
        self.assertFalse(result['ok']);self.assertEqual(bus.writes,[6,9])
        self.assertEqual(result['restored_algorithm'],9);self.assertEqual(mmio.permit,0)

    def test_permission_on_refuses_even_sensor_access(self):
        bus=Bus()
        with self.assertRaises(RuntimeError):ab.compare(MMIO(1),bus,lambda:None)
        self.assertEqual(bus.writes,[])

    def test_wrapped_stationary_yaw_summary(self):
        a=sample();b=dict(a,imu_yaw=-3.13)
        result=ab.summary([a,b])
        self.assertLess(abs(result['yaw_drift_deg']),2)
        self.assertEqual(result['valid'],2)
