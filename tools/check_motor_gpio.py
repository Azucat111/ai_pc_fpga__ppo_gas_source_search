"""Board-only static control-pin check, using the installed PYNQ base overlay.

Disconnect motor power before use. The default paired check changes both
inputs together. Single-pin checks require the expansion board to be removed
and --bare-board confirmation; they must not be used with a connected bridge.
It temporarily replaces the car overlay and does not restore it automatically.
"""
import argparse
import json
import signal
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--motor-power-disconnected', action='store_true', required=True)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument('--channel', choices=('a', 'b'))
    selection.add_argument('--pin', type=int, choices=(3, 5, 6, 9))
    parser.add_argument('--bare-board', action='store_true',
                        help='Confirm expansion board and all driven loads are physically disconnected')
    parser.add_argument('--level', choices=('low', 'high'), default='high')
    parser.add_argument('--seconds', type=int, default=300)
    parser.add_argument('--reuse-loaded-base', action='store_true')
    args = parser.parse_args()
    if args.pin is not None and not args.bare_board:
        parser.error('--pin requires --bare-board; remove the expansion board first')
    if not 5 <= args.seconds <= 600:
        parser.error('--seconds must be 5..600')
    for p in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            command = p.read_bytes()
        except OSError:
            continue
        if b'-m\x00car.board' in command or b'-m\x00car.diagnose' in command:
            raise RuntimeError('Stop the car board/diagnostic process before replacing the overlay: ' + p.parent.name)

    import pynq.overlays.base
    from pynq.overlays.base import BaseOverlay
    from pynq.lib.arduino import Arduino_IO
    from pynq.lib.arduino import ARDUINO_DIO_BASEADDR, ARDUINO_DIO_DATA_OFFSET

    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    bit = Path(pynq.overlays.base.__file__).parent / 'base.bit'
    overlay = BaseOverlay(str(bit), download=not args.reuse_loaded_base)
    info = getattr(overlay, 'ARDUINO', None)
    if not overlay.is_loaded() or not isinstance(info, dict) or info.get('ip_name') != 'iop_arduino/mb_bram_ctrl':
        raise RuntimeError('Installed base overlay does not have the expected Arduino IOP')

    data_address = ARDUINO_DIO_BASEADDR + ARDUINO_DIO_DATA_OFFSET
    bridge = None
    pins = {}
    channel = args.channel or 'a'
    mask = (1 << 3) | (1 << 5) if channel == 'a' else (1 << 6) | (1 << 9)
    if args.pin is not None:
        mask = 1 << args.pin
    motor_mask = (1 << 3) | (1 << 5) | (1 << 6) | (1 << 9)
    try:
        for pin in (3, 5, 6, 9):
            pins[pin] = Arduino_IO(info, pin, 'out')
            bridge = pins[pin].microblaze
            bridge.write_cmd(data_address, 0)
        value = mask if args.level == 'high' else 0
        bridge.write_cmd(data_address, value)
        print(json.dumps({'ready': True, 'channel': channel if args.pin is None else None,
                          'selected_pin': args.pin, 'bare_board_confirmed': args.bare_board,
                          'selected_mask': mask, 'level': args.level,
                          'expected_voltage_v': 3.3 if value else 0,
                          'hold_seconds': args.seconds,
                          'gpio_register_readback': bridge.read_cmd(data_address),
                          'motor_gpio_bits': bridge.read_cmd(data_address) & motor_mask,
                          'pin_map': {'AIN1': 3, 'AIN2': 5, 'BIN1': 6, 'BIN2': 9}}), flush=True)
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            time.sleep(min(.25, max(0, end - time.monotonic())))
    except KeyboardInterrupt:
        pass
    finally:
        if bridge is not None:
            bridge.write_cmd(data_address, 0)
            print(json.dumps({'finished': True,
                              'gpio_register_readback': bridge.read_cmd(data_address),
                              'motor_gpio_bits': bridge.read_cmd(data_address) & motor_mask,
                              'car_overlay_loaded': False}), flush=True)


if __name__ == '__main__':
    main()
