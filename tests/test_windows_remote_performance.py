import sys,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import windows_remote_performance as module
class TestSession(unittest.TestCase):
 def test_ui_exception_still_restores_power(self):
  settings=dict(dc=2)
  with patch.object(sys,'argv',['remote']),patch.object(module,'performance_mode',return_value=settings),patch.object(module.windows_remote,'main',side_effect=RuntimeError('UI failure')),patch.object(module,'restore_power') as restore:
   with self.assertRaises(RuntimeError):module.main()
   restore.assert_called_once_with(settings)
 def test_smoke_never_changes_power(self):
  with patch.object(sys,'argv',['remote','--smoke-test']),patch.object(module,'performance_mode') as power,patch.object(module.windows_remote,'main') as gui:
   module.main();power.assert_not_called();gui.assert_called_once()
 def test_no_power_permission_does_not_prevent_gui(self):
  with patch.object(sys,'argv',['remote']),patch.object(module,'performance_mode',side_effect=OSError('no permission')),patch.object(module.windows_remote,'main') as gui,patch.object(module,'restore_power') as restore:
   module.main();gui.assert_called_once();restore.assert_not_called()
