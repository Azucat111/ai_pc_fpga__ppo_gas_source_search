"""One local GO covers three bounded actions; failures cannot advance the route."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import offline_ground_trial_windows as runner


def ready(boot=123):
    return dict(boot_id=boot,enabled=True,geometry_valid=True,imu_frame_confirmed=True,
                imu_yaw=1.,imu_age_ms=18,wheel_rpm_left=0,wheel_rpm_right=0,
                tgs2602=[dict(valid=True),dict(valid=True)],straight_gyro_heading=True,turn_gyro_heading=True)


class LRouteTests(unittest.TestCase):
    def fake_child(self,failed_step=None):
        def run(state,directory,log,degrees=None,straight_seconds=3,stop_file=None):
            index=int(Path(directory).name.split('_')[-1])
            out=Path(directory)/'trial';out.mkdir(parents=True)
            ok=index!=failed_step
            item=dict(ok=ok,stop_acknowledged=True,boot_id=state['boot_id'])
            (out/'summary.json').write_text(json.dumps(item))
            return dict(child_returncode=0 if ok else 1)
        return run

    def test_no_go_and_old_core_never_launch_route(self):
        with tempfile.TemporaryDirectory() as tmp,patch.object(runner,'execute_motion_step') as child,patch.object(runner,'read_and_stop') as read:
            r=runner.run_l_route(ready(),Path(tmp),{},lambda _:None,lambda _:'')
            self.assertTrue(r['cancelled']);child.assert_not_called();read.assert_not_called()
            state=ready();state['turn_gyro_heading']=False
            with self.assertRaises(RuntimeError):runner.run_l_route(state,Path(tmp),{},lambda _:None,lambda _:'GO')
            child.assert_not_called()

    def test_one_go_runs_ordered_segments_with_fresh_stops(self):
        with tempfile.TemporaryDirectory() as tmp,patch.object(runner,'operator_countdown'),patch.object(runner,'read_and_stop',return_value=ready()) as read,patch.object(runner,'execute_motion_step',side_effect=self.fake_child()) as child:
            prompt=Mock(return_value='GO')
            result=runner.run_l_route(ready(),Path(tmp),{},lambda _:None,prompt)
            self.assertEqual(prompt.call_count,1);self.assertEqual(read.call_count,3)
            self.assertEqual([c.args[3] for c in child.call_args_list],[None,90,None])
            self.assertTrue(all(c.args[4]==6 for c in child.call_args_list))
            self.assertTrue(all(c.kwargs['stop_file']==Path(tmp)/'STOP.request' for c in child.call_args_list))
            self.assertEqual(result['child_returncode'],0)
            self.assertTrue(json.loads((Path(tmp)/'trial/summary.json').read_text())['ok'])

    def test_failed_turn_prevents_second_straight_and_restores_network(self):
        with tempfile.TemporaryDirectory() as tmp:
            wifi=Mock(on_car=True,restore_profile='Normal',cleanup_error=None);wifi.restore.return_value=True
            with patch.object(runner,'CarWiFi',return_value=wifi),patch.object(runner,'read_and_stop',return_value=dict(ready(),last_action='stop')),patch.object(runner,'operator_countdown'),patch.object(runner,'execute_motion_step',side_effect=self.fake_child(failed_step=2)) as child,patch('builtins.print'):
                r=runner.execute_session(Path(tmp)/'session',route_l=True,input_fn=lambda _:'GO')
            self.assertEqual(child.call_count,2);self.assertFalse(r['ok'])
            self.assertTrue(r['final_stop_acknowledged']);self.assertTrue(r['restored'])
            wifi.restore.assert_called_once()
            summary=json.loads((Path(tmp)/'session/trial/summary.json').read_text())
            self.assertFalse(summary['ok']);self.assertEqual(len(summary['steps']),2)

    def test_restart_between_steps_prevents_next_action(self):
        with tempfile.TemporaryDirectory() as tmp,patch.object(runner,'operator_countdown'),patch.object(runner,'read_and_stop',side_effect=[ready(),ready(456)]),patch.object(runner,'execute_motion_step',side_effect=self.fake_child()) as child:
            with self.assertRaisesRegex(RuntimeError,'重启'):
                runner.run_l_route(ready(),Path(tmp),{},lambda _:None,lambda _:'GO')
            self.assertEqual(child.call_count,1)

    def test_stop_file_prevents_first_action(self):
        with tempfile.TemporaryDirectory() as tmp,patch.object(runner,'operator_countdown'),patch.object(runner,'execute_motion_step') as child:
            (Path(tmp)/'STOP.request').touch()
            with self.assertRaises(KeyboardInterrupt):runner.run_l_route(ready(),Path(tmp),{},lambda _:None,lambda _:'GO')
            child.assert_not_called()

    def test_timeout_or_interrupt_does_not_advance_and_still_stops(self):
        for error in (subprocess.TimeoutExpired('phase',25),KeyboardInterrupt()):
            with self.subTest(error=type(error).__name__),tempfile.TemporaryDirectory() as tmp:
                wifi=Mock(on_car=True,restore_profile='Normal',cleanup_error=None);wifi.restore.return_value=True
                with patch.object(runner,'CarWiFi',return_value=wifi),patch.object(runner,'read_and_stop',return_value=dict(ready(),last_action='stop')) as stop,patch.object(runner,'operator_countdown'),patch.object(runner,'execute_motion_step',side_effect=error) as child,patch('builtins.print'):
                    r=runner.execute_session(Path(tmp)/'session',route_l=True,input_fn=lambda _:'GO')
                self.assertEqual(child.call_count,1);self.assertEqual(stop.call_count,3)
                self.assertFalse(r['ok']);self.assertTrue(r['final_stop_acknowledged']);self.assertTrue(r['restored'])
