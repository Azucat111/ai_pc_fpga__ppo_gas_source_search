"""Offline operator confirmation and restoration must survive cancellation/failure."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock,patch
from types import SimpleNamespace

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import offline_ground_trial_windows as runner


def ready():
    return dict(boot_id=123,enabled=True,last_action='stop',geometry_valid=True,
                imu_frame_confirmed=True,imu_yaw=1.5,imu_age_ms=18,
                wheel_rpm_left=0,wheel_rpm_right=0,
                tgs2602=[dict(valid=True),dict(valid=True)])


class OfflineOperatorTests(unittest.TestCase):
    def test_longer_straight_uses_local_go_and_preserves_short_default(self):
        for seconds in (3,6):
            with self.subTest(seconds=seconds),patch.object(runner.subprocess,'run',return_value=SimpleNamespace(returncode=0)) as child,patch.object(runner.time,'sleep'),patch.dict(sys.modules,{'winsound':SimpleNamespace(Beep=lambda *_:None)}):
                kwargs={} if seconds==3 else dict(straight_seconds=6)
                runner.run_ground_trial(ready(),Path('unused'),lambda _:None,lambda _:'GO',**kwargs)
            cmd=child.call_args.args[0]
            self.assertTrue(cmd[3].endswith('check_ground_straight.py'))
            self.assertEqual(cmd[cmd.index('--seconds')+1],str(seconds))
            self.assertEqual(cmd[cmd.index('--rpm')+1],'20')
            self.assertIn('--ground-clear',cmd)
        with patch.object(runner.subprocess,'run') as child:
            result=runner.run_ground_trial(ready(),Path('unused'),lambda _:None,lambda _:'',straight_seconds=6)
        self.assertTrue(result['cancelled']);child.assert_not_called()

    def test_stationary_cancel_never_launches_child_and_restores(self):
        with tempfile.TemporaryDirectory() as tmp:
            wifi=Mock(on_car=True,restore_profile='Normal',cleanup_error=None)
            wifi.restore.return_value=True
            with patch.object(runner,'CarWiFi',return_value=wifi),patch.object(runner,'read_and_stop',return_value=ready()),patch.object(runner.subprocess,'run') as child,patch('builtins.print'):
                result=runner.execute_session(Path(tmp)/'session',input_fn=lambda _:'',observe_still=True)
            child.assert_not_called();self.assertTrue(result['cancelled'])
            self.assertTrue(result['final_stop_acknowledged']);self.assertTrue(result['restored'])

    def test_no_go_never_launches_a_motion_child(self):
        with patch.object(runner.subprocess,'run') as child:
            result=runner.run_ground_trial(ready(),Path('unused'),lambda _:None,lambda _:'')
        self.assertTrue(result['cancelled']);child.assert_not_called()

    def test_turn_entry_selects_turn_controller_only_after_local_go(self):
        for degrees in (-90,-30,30,90):
            with self.subTest(degrees=degrees),patch.object(runner.subprocess,'run',return_value=SimpleNamespace(returncode=0)) as child,patch.object(runner.time,'sleep'),patch.dict(sys.modules,{'winsound':SimpleNamespace(Beep=lambda *_:None)}):
                result=runner.run_ground_trial(ready(),Path('unused'),lambda _:None,lambda _:'GO',degrees)
            cmd=child.call_args.args[0]
            self.assertTrue(cmd[3].endswith('check_ground_turn.py'))
            self.assertEqual(cmd[cmd.index('--degrees')+1],str(degrees))
            self.assertEqual(cmd[cmd.index('--seconds')+1],'10')
            self.assertIn('--ground-clear',cmd);self.assertNotIn('--rpm',cmd)
            self.assertEqual(result['child_returncode'],0)

    def test_invalid_preflight_cannot_reach_operator_start_or_child(self):
        for change in (dict(enabled=False),dict(imu_yaw=None),dict(imu_age_ms=151),
                       dict(wheel_rpm_left=3),dict(tgs2602=[dict(valid=False)])):
            state=copy.deepcopy(ready());state.update(change)
            with self.subTest(change=change),patch.object(runner.subprocess,'run') as child:
                with self.assertRaises(RuntimeError):
                    runner.run_ground_trial(state,Path('unused'),lambda _:None,Mock(side_effect=AssertionError('Must not prompt')))
                child.assert_not_called()

    def test_connect_failure_still_restores_and_cleans_temporary_profile(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);calls=[]
            def netsh(*args):
                calls.append(args)
                if args[0]=='connect' and args[1]!='name=Normal':raise RuntimeError('injected connect failure')
                return ''
            state=dict(profile='Normal',ssid='Home',interface='WLAN')
            with patch.object(runner,'wifi_state',return_value=state),patch.object(runner,'credentials',return_value=('FPGA-CAR','test-key')),patch.object(runner,'netsh',side_effect=netsh),patch.object(runner.time,'sleep'):
                wifi=runner.CarWiFi(root/'settings.json',root/'build')
                try:
                    with self.assertRaises(RuntimeError):wifi.connect()
                finally:restored=wifi.restore()
            self.assertTrue(restored)
            self.assertIn(('connect','name=Normal','interface=WLAN'),calls)
            self.assertTrue(any(c[:2]==('delete','profile') for c in calls))
            self.assertFalse(wifi.xml.exists())
            self.assertNotIn('test-key',(root/'settings.json').read_text())

    def test_already_connected_to_car_uses_saved_normal_profile(self):
        with tempfile.TemporaryDirectory() as tmp:
            settings=Path(tmp)/'settings.json';settings.write_text(json.dumps(dict(profile='Normal',interface='WLAN')))
            state=dict(profile='Manual-car',ssid='FPGA-CAR',interface='WLAN')
            with patch.object(runner,'credentials',return_value=('FPGA-CAR','test-key')),patch.object(runner,'wifi_state',side_effect=[state,dict(profile='Normal')]),patch.object(runner,'netsh') as netsh,patch.object(runner.time,'sleep'):
                wifi=runner.CarWiFi(settings,Path(tmp));wifi.connect()
                netsh.assert_not_called()
                self.assertTrue(wifi.restore())
            netsh.assert_called_once_with('connect','name=Normal','interface=WLAN')

    def test_child_timeout_or_ctrl_c_still_requests_stop_and_restores_network(self):
        for error in (subprocess.TimeoutExpired('trial',25),KeyboardInterrupt()):
            with self.subTest(error=type(error).__name__),tempfile.TemporaryDirectory() as tmp:
                wifi=Mock(on_car=True,restore_profile='Normal',cleanup_error=None)
                wifi.restore.return_value=True
                with patch.object(runner,'CarWiFi',return_value=wifi),patch.object(runner,'read_and_stop',return_value=ready()) as stop,patch.object(runner,'run_ground_trial',side_effect=error),patch('builtins.print'):
                    result=runner.execute_session(Path(tmp)/'session',input_fn=lambda _:'GO')
                self.assertFalse(result['ok']);self.assertTrue(result['restored'])
                self.assertTrue(result['final_stop_acknowledged'])
                self.assertEqual(stop.call_count,2);wifi.restore.assert_called_once()
                self.assertTrue((Path(tmp)/'session/RESULT.txt').is_file())
