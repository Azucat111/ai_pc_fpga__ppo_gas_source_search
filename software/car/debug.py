"""Read motor diagnostics from an already loaded DBG1-capable car overlay."""
import argparse
import json
import math
import time
from .protocol import signed32

ADDRESSES = (0x08, 0x10, 0x14, 0x18, 0x1c, 0x20, 0x24,
             0x64, 0x68, 0x6c, 0x70, 0x74, 0x78, 0x7c,
             0xa0, 0xa4, 0xa8, 0xac, 0xb0, 0xb4, 0xb8, 0xbc, 0xc0, 0xc4, 0xc8, 0xcc, 0xd0, 0xd4, 0xd8)
PIN_NAMES = ('AIN1', 'AIN2', 'BIN1', 'BIN2')


def signed16(value):
    value &= 0xffff
    return value - 0x10000 if value & 0x8000 else value


def decode_registers(words):
    if words[0x64] != 0x44424731:
        raise RuntimeError('DBG1 diagnostics missing; load the diagnostic overlay first')
    duties = lambda value: {'left': signed16(value), 'right': signed16(value >> 16)}
    targets = lambda left, right: {'left': signed32(left) / 65536, 'right': signed32(right) / 65536}
    flags = words[0x7c]
    return {
        'timestamp_ms': words[0x08], 'boot_id': words[0x10],
        'motor_permission': bool(words[0x14] & 1),
        'declared_clock_hz': words[0x68], 'control_ticks': words[0x6c],
        'wake_interval_us': words[0xc8],
        'imu_last_good_supported': words[0xcc] in (0x494d5531, 0x494d5532),
        'imu_plausibility_supported': words[0xcc] == 0x494d5532,
        'gas10_supported': words.get(0xd8)==0x47415331,
        'turn_gyro_supported': words.get(0xd4)==0x47595232,
        'straight_gyro_supported': words.get(0xd0)==0x47595231,
        'wiring_flags': words[0x18],
        'kp': words[0x1c] / 256, 'ki_per_tick': words[0x20] / 256,
        'duty_limit_permille': words[0x24] & 0xffff,
        'deadzone_permille': words[0x24] >> 16,
        'target_counts_per_10ms': targets(words[0x70], words[0x74]),
        'duty_permille': duties(words[0x78]), 'drive_active': bool(flags & 16),
        'output_logic': {name: bool(flags & (1 << i)) for i, name in enumerate(PIN_NAMES)},
        'last_received_motion': {'mode': words[0xa0], 'p0': signed32(words[0xa4]), 'p1': signed32(words[0xa8])},
        'last_nonzero_duty_permille': duties(words[0xac]),
        'output_rising_edges': {name: words[0xb0 + 4*i] for i, name in enumerate(PIN_NAMES)},
        'last_active_target_counts_per_10ms': targets(words[0xc0], words[0xc4])}


def read_diagnostics(mmio):
    if mmio.read(0) != 0x43415231:
        raise RuntimeError('CAR1 signature missing; diagnostic reader did not load/reconfigure hardware')
    if mmio.read(0x64) != 0x44424731:
        raise RuntimeError('The loaded car overlay predates DBG1 diagnostics')
    return decode_registers({address: mmio.read(address) for address in ADDRESSES})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--interval', type=float, default=.2)
    parser.add_argument('--once', action='store_true')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 600:
        parser.error('--seconds must be 0..600')
    if not math.isfinite(args.interval) or not .02 <= args.interval <= 1:
        parser.error('--interval must be 0.02..1')
    from pynq import MMIO, Clocks
    mmio = MMIO(0x43c00000, 0x1000)
    print(json.dumps({'fclk0_mhz': Clocks.fclk0_mhz, 'access': 'read_only'}), flush=True)
    end = time.monotonic() + args.seconds
    while True:
        print(json.dumps(read_diagnostics(mmio)), flush=True)
        if args.once or time.monotonic() >= end:
            break
        time.sleep(args.interval)


if __name__ == '__main__':
    main()
