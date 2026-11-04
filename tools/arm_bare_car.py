"""Temporarily arm car signal checks or a bounded raised-wheel motor pulse.

Never modifies config/car.json. Skips absent sensors and uses bounded gains.
Signal checks require disconnected loads. Driver-no-load can use VM only with
both motors unplugged. Motor-pulse requires raised wheels; default cap is 150
permille, with an explicit 700 permille diagnostic option.
"""
import argparse
import json
from pathlib import Path
import signal
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    setup = p.add_mutually_exclusive_group(required=True)
    setup.add_argument('--bare-board', action='store_true')
    setup.add_argument('--motor-module-logic-only', action='store_true')
    setup.add_argument('--driver-no-load', action='store_true')
    setup.add_argument('--motor-pulse', action='store_true')
    p.add_argument('--motor-power-disconnected', action='store_true')
    p.add_argument('--motors-disconnected', action='store_true')
    p.add_argument('--wheels-raised', action='store_true')
    p.add_argument('--pulse-duty-permille', type=int, choices=(150, 700), default=150)
    p.add_argument('--seconds', type=int, default=600)
    args = p.parse_args()
    if args.pulse_duty_permille != 150 and not args.motor_pulse:
        p.error('--pulse-duty-permille requires --motor-pulse')
    if args.motor_module_logic_only and not args.motor_power_disconnected:
        p.error('--motor-module-logic-only requires --motor-power-disconnected')
    if args.driver_no_load and (not args.motors_disconnected or args.seconds > 180):
        p.error('--driver-no-load requires --motors-disconnected and --seconds <= 180')
    if args.motor_pulse and (not args.wheels_raised or args.seconds > 30):
        p.error('--motor-pulse requires --wheels-raised and --seconds <= 30')
    if not 5 <= args.seconds <= 600:
        p.error('--seconds must be 5..600')
    from pynq import MMIO, Clocks
    from car.board import configure
    from car.config import load_config
    from car.debug import read_diagnostics
    for f in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            command = f.read_bytes()
        except OSError:
            continue
        if any(x in command for x in (b'-m\x00car.board', b'-m\x00car.diagnose', b'check_bare_pwm', b'check_motor_gpio')):
            raise RuntimeError('Conflicting process: ' + f.parent.name)
    mmio = MMIO(0x43c00000, 0x1000)
    # Check signature before touching permission; base overlay uses another map.
    before = read_diagnostics(mmio)
    if before['declared_clock_hz'] != 5000000 or abs(Clocks.fclk0_mhz - 5) > .01:
        raise RuntimeError('Car overlay clock mismatch')
    if args.motor_pulse and mmio.read(0xc8) != 250:
        raise RuntimeError('Motor pulse requires the v0.1.4 wake-capable bitstream')
    c = load_config('config/car.json')
    if c['m1_is_left'] is not False:
        raise RuntimeError('This probe expects M1=right')
    if not args.motor_pulse and (c['motor_m1_invert'] or c['motor_m2_invert']):
        raise RuntimeError('Raw signal probes expect uninverted motor outputs')
    kp, limit = (1000, 1000) if args.driver_no_load else (
        (1000 if args.pulse_duty_permille == 700 else 100), args.pulse_duty_permille
    ) if args.motor_pulse else (100, 250)
    setup_name = 'motor_pulse' if args.motor_pulse else 'driver_no_load' if args.driver_no_load else 'module_logic_only' if args.motor_module_logic_only else 'bare_board'
    c.update(wheel_kp=kp, wheel_ki_per_tick=0, duty_limit_permille=limit,
             deadzone_permille=0, imu_mount_confirmed=False)
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    records = []
    try:
        boot = configure(mmio, c, enable=False)
        mmio.write(0x98, 0)
        # Signal-test permission does not assert vehicle calibration.
        mmio.write(0x14, 1)
        print(json.dumps({'ready': True, 'boot_id': boot, 'seconds': args.seconds,
                          'temporary_signal_test_permission': True,
                          'setup': setup_name,
                          'kp': kp, 'ki': 0, 'pwm_limit_permille': limit}), flush=True)
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            records.append(read_diagnostics(mmio))
            time.sleep(.2)
    except KeyboardInterrupt:
        pass
    finally:
        mmio.write(0x14, 0)
        time.sleep(.03)
        final = read_diagnostics(mmio)
        out = Path('reports/bare_car_uart_diagnostics.json')
        out.parent.mkdir(exist_ok=True)
        out.write_text(json.dumps({'samples': records, 'final': final}, indent=2) + '\n')
        print(json.dumps({'finished': True, 'final': final, 'report': str(out)}), flush=True)


if __name__ == '__main__':
    main()
