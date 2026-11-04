"""Temporarily prepare a bounded, raised-wheel encoder PI trial; no FPGA reload."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--wheels-raised', action='store_true', required=True)
    p.add_argument('--seconds', type=int, default=30)
    p.add_argument('--duty-limit-permille', type=int, choices=(250, 400, 550, 700), default=400)
    p.add_argument('--kp', type=float, default=10)
    p.add_argument('--ki', type=float, default=1)
    p.add_argument('--deadzone-permille', type=int, default=0)
    a = p.parse_args()
    if not 5 <= a.seconds <= 60:
        p.error('--seconds must be 5..60')
    if not math.isfinite(a.kp) or not 0 < a.kp <= 100:
        p.error('--kp must be finite and in (0, 100]')
    if not math.isfinite(a.ki) or not 0 < a.ki <= 5:
        p.error('--ki must be finite and in (0, 5]')
    if not 0 <= a.deadzone_permille <= min(400, a.duty_limit_permille):
        p.error('deadzone must be 0..min(400, duty limit)')
    from pynq import Clocks, MMIO
    from car.board import configure
    from car.config import load_config, registers
    from car.debug import read_diagnostics
    # sudo wrappers contain this script's name too; exclude our ancestor chain.
    ancestors = set()
    pid = os.getpid()
    while pid > 1:
        ancestors.add(str(pid))
        status = Path('/proc', str(pid), 'status').read_text()
        pid = int(next(line.split()[1] for line in status.splitlines() if line.startswith('PPid:')))
    for f in Path('/proc').glob('[0-9]*/cmdline'):
        if f.parent.name in ancestors:
            continue
        try:
            cmd = f.read_bytes()
        except OSError:
            continue
        if any(x in cmd for x in (b'-m\x00car.board', b'-m\x00car.diagnose',
                                  b'arm_bare_car.py', b'arm_wheel_loop.py',
                                  b'check_bare_pwm', b'check_motor_gpio')):
            raise RuntimeError('Conflicting process: ' + f.parent.name)
    mmio = MMIO(0x43c00000, 0x1000)
    before = read_diagnostics(mmio)
    if (before['declared_clock_hz'] != 5000000 or before['wake_interval_us'] != 250
            or abs(Clocks.fclk0_mhz - 5) > .01):
        raise RuntimeError('Expected loaded v0.1.4 car overlay at 5MHz')
    if before['motor_permission'] or before['drive_active'] or any(before['duty_permille'].values()):
        raise RuntimeError('Stop and disable existing controller before this trial')
    original = load_config('config/car.json')
    if original['m1_is_left'] is not False:
        raise RuntimeError('This trial expects M1=right')
    c = dict(original, wheel_kp=a.kp, wheel_ki_per_tick=a.ki,
             duty_limit_permille=a.duty_limit_permille, deadzone_permille=a.deadzone_permille,
             max_wheel_rpm=80, imu_mount_confirmed=False)
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    samples = []
    boot = None
    try:
        boot = configure(mmio, c, enable=False)
        mmio.write(0x14, 1)
        print(json.dumps(dict(ready=True, boot_id=boot, seconds=a.seconds,
                              setup='encoder_pi_trial', kp=a.kp, ki=a.ki,
                              duty_limit_permille=a.duty_limit_permille,
                              deadzone_permille=a.deadzone_permille)), flush=True)
        end = time.monotonic() + a.seconds
        while time.monotonic() < end:
            samples.append(read_diagnostics(mmio))
            time.sleep(.02)
    except KeyboardInterrupt:
        pass
    finally:
        mmio.write(0x14, 0)
        time.sleep(.03)
        stopped = read_diagnostics(mmio)
        # Restore normal parameters while permission is off; keep trial boot ID/counts.
        for address, value in registers(original).items():
            mmio.write(address, value)
        final = read_diagnostics(mmio)
        out = Path('reports') / ('wheel_loop_' + str(boot) + '.json')
        out.parent.mkdir(exist_ok=True)
        out.write_text(json.dumps(dict(samples=samples, stopped=stopped, final=final), indent=2) + '\n')
        print(json.dumps(dict(finished=True, final=final, report=str(out))), flush=True)


if __name__ == '__main__':
    main()
