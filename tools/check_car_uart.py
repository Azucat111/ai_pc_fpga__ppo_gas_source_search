"""Use a 3.3V USB-TTL adapter to test the existing car UART protocol.

Defaults to STOP/telemetry only. Active probes require an explicit setup and
expected boot ID. Motor pulses require raised wheels. The default low-duty pulse
lasts at most 2 seconds; explicit 700 permille diagnostics allow up to 5 seconds.
Reuses production command, heartbeat and STOP handling.
"""
import argparse
import json
from pathlib import Path
import socket
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'software'))
from car.config import load_config, rpm_target
from car.host import Client
from car.protocol import crc16


class SerialSocket:
    def __init__(self, port, baud=460800):
        import serial
        self.serial = serial.Serial(port, baud, timeout=.02, write_timeout=.5,
                                    bytesize=8, parity='N', stopbits=1)
        self.serial.dtr = False
        self.serial.rts = False
        self.timeout = .2
        self.buffer = bytearray()
        self.frames = 0
        self.crc_errors = 0
        self.discarded = 0

    def settimeout(self, timeout):
        self.timeout = timeout

    def sendto(self, packet, _address):
        if packet != b'FCAR?':
            if self.serial.write(packet) != len(packet):
                raise OSError('Incomplete serial command write')

    def recvfrom(self, _length):
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            self.buffer.extend(self.serial.read(min(4096, max(1, self.serial.in_waiting))))
            while True:
                i = self.buffer.find(b'\xa5\x5a\x01\x81')
                if i < 0:
                    drop = max(0, len(self.buffer) - 3)
                    self.discarded += drop
                    del self.buffer[:drop]
                    break
                if i:
                    self.discarded += i
                    del self.buffer[:i]
                if len(self.buffer) < 192:
                    break
                packet = bytes(self.buffer[:192])
                if crc16(packet[:-2]) != int.from_bytes(packet[-2:], 'little'):
                    self.crc_errors += 1
                    del self.buffer[0]
                    continue
                del self.buffer[:192]
                self.frames += 1
                return packet, ('serial', 0)
        raise socket.timeout('No complete UART telemetry frame')

    def close(self):
        self.serial.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--config', default=str(ROOT / 'config/car.json'))
    parser.add_argument('--probe', action='store_true')
    setup = parser.add_mutually_exclusive_group()
    setup.add_argument('--bare-board', action='store_true')
    setup.add_argument('--motor-module-logic-only', action='store_true')
    setup.add_argument('--driver-no-load', action='store_true')
    setup.add_argument('--motor-pulse', action='store_true')
    parser.add_argument('--motor-power-disconnected', action='store_true')
    parser.add_argument('--motors-disconnected', action='store_true')
    parser.add_argument('--wheels-raised', action='store_true')
    parser.add_argument('--pulse-duty-permille', type=int, choices=(150, 700), default=150)
    parser.add_argument('--reverse', action='store_true', help='Reverse command for driver-no-load or motor-pulse')
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument('--both-channels', action='store_true', help='Both bridges, driver-no-load only')
    selection.add_argument('--bridge', choices=('a', 'b'), default='a', help='Physical bridge to test; B requires driver-no-load or motor-pulse')
    parser.add_argument('--boot-id', type=int)
    parser.add_argument('--seconds', type=float, default=300)
    parser.add_argument('--stop-file', type=Path,
                        help='Create this file to request a graceful STOP without killing the process')
    args = parser.parse_args()
    if args.pulse_duty_permille != 150 and not args.motor_pulse:
        parser.error('--pulse-duty-permille requires --motor-pulse')
    pulse_max_seconds = 5 if args.pulse_duty_permille == 700 else 2
    if args.motor_pulse and (not args.wheels_raised or not .1 <= args.seconds <= pulse_max_seconds):
        parser.error(f'--motor-pulse requires --wheels-raised and --seconds 0.1..{pulse_max_seconds}')
    if not args.motor_pulse and not 5 <= args.seconds <= 600:
        parser.error('--seconds must be 5..600')
    if args.motor_module_logic_only and not args.motor_power_disconnected:
        parser.error('--motor-module-logic-only requires --motor-power-disconnected')
    if args.driver_no_load and (not args.motors_disconnected or args.seconds > 120):
        parser.error('--driver-no-load requires --motors-disconnected and --seconds <= 120')
    if args.reverse and not (args.driver_no_load or args.motor_pulse):
        parser.error('--reverse requires --driver-no-load or --motor-pulse')
    if args.both_channels and not args.driver_no_load:
        parser.error('--both-channels is only available with --driver-no-load')
    if args.bridge == 'b' and not (args.driver_no_load or args.motor_pulse):
        parser.error('--bridge b requires --driver-no-load or --motor-pulse')
    if args.probe and (not (args.bare_board or args.motor_module_logic_only or args.driver_no_load or args.motor_pulse) or args.boot_id is None):
        parser.error('--probe requires a signal-test setup and --boot-id')
    if args.stop_file is not None and args.stop_file.exists():
        parser.error('--stop-file already exists; use a fresh path')
    config = load_config(args.config)
    setup_name = 'motor_pulse' if args.motor_pulse else 'driver_no_load' if args.driver_no_load else 'module_logic_only' if args.motor_module_logic_only else 'bare_board' if args.bare_board else 'passive'
    target_rpm = -10 if args.reverse else 10
    left_rpm = target_rpm if args.both_channels or args.bridge == 'b' else 0
    right_rpm = target_rpm if args.both_channels or args.bridge == 'a' else 0
    expected_control = ('AIN2 and BIN2 high, AIN1 and BIN1 low' if args.reverse else
                        'AIN1 and BIN1 high, AIN2 and BIN2 low') if args.both_channels else (
                        'BIN2 high, BIN1 low' if args.reverse else 'BIN1 high, BIN2 low') if args.bridge == 'b' else (
                        'AIN2 high, AIN1 low' if args.reverse else 'AIN1 high, AIN2 low' if args.driver_no_load else 'AIN1 PWM, AIN2 low')
    if args.motor_pulse:
        prefix = 'A' if args.bridge == 'a' else 'B'
        inverted = config['motor_m1_invert' if args.bridge == 'a' else 'motor_m2_invert']
        expected_control = prefix + ('IN2 PWM, IN1 low' if args.reverse ^ inverted else 'IN1 PWM, IN2 low')
    transport = SerialSocket(args.port)
    baseline = None
    active_announced = False
    cancel_requested = False
    records = []

    class CapturingClient(Client):
        def receive(self, *values, **kwargs):
            nonlocal active_announced, cancel_requested
            item = super().receive(*values, **kwargs)
            records.append(item)
            if args.stop_file is not None and args.stop_file.exists() and not cancel_requested:
                cancel_requested = True
                raise KeyboardInterrupt
            if args.probe and baseline is not None:
                if item['boot_id'] != args.boot_id:
                    raise RuntimeError('Board restarted during probe')
                checked_sides = (('left',) if args.bridge == 'a' else ('right',)) if args.motor_pulse else ('left', 'right')
                if any(abs(item['encoder_' + side + '_count'] - baseline['encoder_' + side + '_count']) > 2
                       for side in checked_sides):
                    raise RuntimeError('Unexpected encoder activity during disconnected-load probe')
                if item['last_action'] == 'wheel_velocity' and not active_announced:
                    active_announced = True
                    print(json.dumps({'active': True, 'boot_id': item['boot_id'],
                                      'device_timestamp_ms': item['timestamp_ms'],
                                      'mode': item['last_action'], 'right_target_rpm': right_rpm,
                                      'left_target_rpm': left_rpm,
                                      'setup': setup_name,
                                      'expected_duty_percent': (args.pulse_duty_permille * 250 // 1000) / 2.5 if args.motor_pulse else 100 if args.driver_no_load else 21.6,
                                      'expected_control': expected_control}), flush=True)
            return item

    client = CapturingClient('serial', config, port=0, sock=transport)
    summary = {'port': args.port, 'probe': args.probe, 'ok': False,
               'setup': setup_name, 'target_right_rpm': right_rpm, 'target_left_rpm': left_rpm}
    if args.motor_pulse:
        summary.update(pulse_duty_permille=args.pulse_duty_permille, requested_seconds=args.seconds)
    try:
        first = client.receive(timeout=5)
        if args.boot_id is not None and first['boot_id'] != args.boot_id:
            raise RuntimeError('Wrong board boot ID')
        stop = client.send(0)
        summary.update(boot_id=stop['boot_id'], stop_acknowledged=stop['last_action'] == 'stop')
        print(json.dumps({'uart_ready': True, 'boot_id': stop['boot_id'], 'enabled': stop['enabled'],
                          'stop_acknowledged': summary['stop_acknowledged']}), flush=True)
        if args.probe:
            baseline = stop
            client.hold(1, rpm_target(left_rpm, config, 'left'), rpm_target(right_rpm, config, 'right'), args.seconds)
        else:
            for _ in range(10):
                client.receive()
        summary['ok'] = True
    except KeyboardInterrupt:
        summary.update(cancelled=True, ok=active_announced if args.probe else False)
    finally:
        try:
            stop = client.send(0)
            summary['final_action'] = stop['last_action']
        except (OSError, RuntimeError, TimeoutError) as exc:
            summary['stop_error'] = str(exc)
        client.close()
        summary.update(frames=transport.frames, crc_errors=transport.crc_errors,
                       discarded_bytes=transport.discarded)
        output = ROOT / 'build/output'
        prefix = 'bare_car_uart_probe' if args.probe else 'bare_car_uart_preflight'
        (output / (prefix + '.jsonl')).write_text(''.join(json.dumps(x) + '\n' for x in records), encoding='utf-8')
        (output / (prefix + '_summary.json')).write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
        print(json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
