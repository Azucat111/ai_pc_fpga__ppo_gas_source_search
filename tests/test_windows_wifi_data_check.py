"""Windows passive data entry restores Wi-Fi on success/failure without sending STOP."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock,patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import windows_wifi_data_check as checker


def captured():
    return dict(transport_check_passed=True,unique_frames=20,valid_frames_including_duplicates=20,
                invalid_datagrams=0,router_replies={},boots=[dict(boot_id=123)],
                fresh_valid_frame_count=dict(imu=20,bme=20,tgs0=20,tgs1=20),bme_gas_usable_frames=20)


class WindowsWiFiDataCheckTests(unittest.TestCase):
    def test_success_restores_wifi_and_only_calls_passive_collector(self):
        with tempfile.TemporaryDirectory() as tmp:
            wifi=Mock(restore_profile='Normal');wifi.restore.return_value=True
            with patch.object(checker,'CarWiFi',return_value=wifi),patch.object(checker,'collect',return_value=captured()) as capture,patch.object(checker.socket,'socket') as probe,patch('builtins.print'):
                probe.return_value.__enter__.return_value.getsockname.return_value=('192.168.4.2',1234)
                r=checker.run_check(Path(tmp)/'result',20)
            wifi.connect.assert_called_once();wifi.restore.assert_called_once();capture.assert_called_once()
            self.assertTrue(r['ok']);self.assertEqual(r['car_motion_commands_sent'],0);self.assertEqual(r['car_stop_commands_sent'],0)
            self.assertEqual(r['route_source_ip'],'192.168.4.2')
            self.assertTrue((Path(tmp)/'result/RESULT.txt').exists())

    def test_connection_failure_still_restores_without_capturing(self):
        with tempfile.TemporaryDirectory() as tmp:
            wifi=Mock(restore_profile='Normal');wifi.connect.side_effect=RuntimeError('no AP');wifi.restore.return_value=True
            with patch.object(checker,'CarWiFi',return_value=wifi),patch.object(checker,'collect') as capture,patch('builtins.print'):
                r=checker.run_check(Path(tmp)/'result')
            capture.assert_not_called();wifi.restore.assert_called_once()
            self.assertFalse(r['ok']);self.assertTrue(r['restored']);self.assertIn('no AP',r['error'])

    def test_capture_interrupt_or_error_always_restores(self):
        for error in (KeyboardInterrupt(),OSError('capture failed')):
            with self.subTest(error=type(error).__name__),tempfile.TemporaryDirectory() as tmp:
                wifi=Mock(restore_profile='Normal');wifi.restore.return_value=True
                with patch.object(checker,'CarWiFi',return_value=wifi),patch.object(checker,'collect',side_effect=error),patch.object(checker.socket,'socket') as probe,patch('builtins.print'):
                    probe.return_value.__enter__.return_value.getsockname.return_value=('192.168.4.2',1234)
                    r=checker.run_check(Path(tmp)/'result')
                self.assertTrue(r['restored']);wifi.restore.assert_called_once();self.assertFalse(r['ok'])

    def test_zero_frames_and_invalid_sensors_are_not_confused(self):
        self.assertIn('没有收到',checker.explain({}))
        self.assertIn('FCAR_FULL',checker.explain(dict(router_replies={'FCAR_FULL':1})))
        c=captured();c['fresh_valid_frame_count']={k:0 for k in c['fresh_valid_frame_count']}
        self.assertIn('状态帧已收到',checker.explain(c))
        c=captured();c['bme_gas_usable_frames']=0
        self.assertIn('BME气体电阻',checker.explain(c))
        self.assertIn('本机已收到',checker.explain(captured()))

    def test_gas10_check_requires_valid_scans_not_just_state_packets(self):
        for scans,expected in ((0,False),(2,False),(3,True)):
            with self.subTest(scans=scans),tempfile.TemporaryDirectory() as tmp:
                wifi=Mock(restore_profile='Normal');wifi.restore.return_value=True
                sample=dict(captured(),gas10_frames=scans,gas10_valid_unique_scans=scans,bme_gas_usable_frames=0)
                with patch.object(checker,'CarWiFi',return_value=wifi),patch.object(checker,'collect',return_value=sample),patch.object(checker.socket,'socket') as probe,patch('builtins.print'):
                    probe.return_value.__enter__.return_value.getsockname.return_value=('192.168.4.2',1234)
                    r=checker.run_check(Path(tmp)/'result',30,True)
                self.assertEqual(r['ok'],expected);self.assertEqual(r['car_motion_commands_sent'],0)
                self.assertTrue(r['restored']);wifi.restore.assert_called_once()
        self.assertIn('GAS10',checker.explain(dict(captured(),gas10_valid_unique_scans=3,bme_gas_usable_frames=0)))
