"""Record inputs and artifacts only after a fresh timing-closed Vivado build."""
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'software'))
from car import __version__


def main():
    inputs = sorted((ROOT / 'rtl').glob('*.sv')) + [ROOT / name for name in
              ('vivado/build.tcl', 'vivado/car.xdc', 'vendor/pynq/pynqz2.tcl')]
    outputs = [ROOT / 'build/output' / name for name in
               ('car.bit', 'car.hwh', 'timing_summary.rpt', 'utilization.rpt', 'drc.rpt', 'io.rpt')]
    outputs.append(ROOT / 'build/hardware_export/car.xsa')
    if not all(p.is_file() for p in inputs + outputs):
        raise SystemExit('Build inputs or outputs missing')
    timing = ROOT / 'build/output/timing_summary.rpt'
    if 'All user specified timing constraints are met.' not in timing.read_text():
        raise SystemExit('Timing is not closed')
    if timing.stat().st_mtime < max(p.stat().st_mtime for p in inputs):
        raise SystemExit('Timing report predates build inputs; refusing stale record')
    def hashes(paths):
        return {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    record = {'version': __version__, 'vivado': '2022.2', 'debug_signature': '0x44424731',
              'clock_hz': 5000000, 'wake_interval_us': 250,
              'imu_last_good_signature': '0x494d5532',
              'straight_gyro_signature': '0x47595231',
              'turn_gyro_signature': '0x47595232',
              'gas10_signature': '0x47415331',
              'source_led_signature': '0x4c454431',
              'inputs': hashes(inputs), 'artifacts': hashes(outputs)}
    path = ROOT / 'build/output/build_manifest.json'
    path.write_text(json.dumps(record, indent=2) + '\n')
    print(path)
    print('Recorded', len(inputs), 'build inputs and', len(outputs), 'artifacts for', __version__)


if __name__ == '__main__':
    main()
