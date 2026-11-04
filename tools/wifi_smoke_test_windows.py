"""Read car AP telemetry, optionally probe STOP, then restore Wi-Fi."""
import argparse
import json
import locale
import re
import socket
import subprocess
import sys
import time
import uuid
from pathlib import Path
from xml.sax.saxutils import escape

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'software'))
from car.config import load_config, rpm_target
from car.protocol import decode, command
from car.host import Client


def netsh(*args):
    result = subprocess.run(['netsh', 'wlan', *args], capture_output=True,
                            timeout=8)
    output = result.stdout.decode(locale.getpreferredencoding(False), 'replace')
    if result.returncode:
        raise RuntimeError(output.strip() or f'netsh failed: {result.returncode}')
    return output


def current_profile():
    output = netsh('show', 'interfaces')
    matches = re.findall(r'^\s*(?:Profile|配置文件)\s*:\s*(.+?)\s*$', output, re.M)
    if len(matches) != 1:
        raise RuntimeError('This tool requires exactly one connected Wi-Fi interface')
    return matches[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', default='WLAN')
    parser.add_argument('--restore-profile', required=True)
    parser.add_argument('--config', default=str(ROOT / 'config/car.json'))
    parser.add_argument('--stop-probe', action='store_true',
                        help='Also verify the return path with STOP only; never enable motion')
    parser.add_argument('--expect-enabled', action='store_true',
                        help='STOP-only check of a board already allowing Wi-Fi commands; sends no motion')
    parser.add_argument('--wheel-probe', choices=('left', 'right'),
                        help='Explicit armed-board test: one wheel at the bounded probe speed/duration, then STOP')
    parser.add_argument('--boot-id', type=int, help='Expected freshly configured boot ID for a wheel probe')
    parser.add_argument('--probe-rpm', type=int, choices=(5, 10), default=5)
    parser.add_argument('--probe-seconds', type=float, default=.5)
    parser.add_argument('--pwm-pin-check', action='store_true',
                        help='Longer signal check only with motor power disconnected; abort if encoders move')
    parser.add_argument('--motor-power-disconnected', action='store_true')
    args = parser.parse_args()
    if sys.platform != 'win32':
        parser.error('Windows only')
    if args.expect_enabled and (not args.stop_probe or args.wheel_probe):
        parser.error('--expect-enabled requires --stop-probe and cannot request a wheel probe')
    if args.wheel_probe and (args.stop_probe or not args.boot_id):
        parser.error('--wheel-probe requires --boot-id and cannot be combined with --stop-probe')
    if args.pwm_pin_check:
        if not args.wheel_probe or not args.motor_power_disconnected or not 5 <= args.probe_seconds <= 60:
            parser.error('--pwm-pin-check requires --wheel-probe, --motor-power-disconnected and 5..60 seconds')
    elif args.probe_seconds not in (.5, 1.0):
        parser.error('Powered wheel probes must be 0.5 or 1 second')
    if current_profile() != args.restore_profile:
        raise RuntimeError('Current Wi-Fi profile differs from --restore-profile')
    config = load_config(args.config)
    output = ROOT / 'build/output'
    output.mkdir(parents=True, exist_ok=True)
    report_prefix = 'pwm_pin_check' if args.pwm_pin_check else (args.wheel_probe + '_probe' if args.wheel_probe else 'wifi')
    profile = 'FPGA-CAR-test-' + uuid.uuid4().hex[:12]
    xml = ROOT / 'build' / (profile + '.xml')
    xml.write_text(f'''<?xml version="1.0"?>
<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
 <name>{escape(profile)}</name><SSIDConfig><SSID><name>FPGA-CAR</name></SSID></SSIDConfig>
 <connectionType>ESS</connectionType><connectionMode>manual</connectionMode>
 <MSM><security><authEncryption><authentication>WPA2PSK</authentication>
 <encryption>AES</encryption><useOneX>false</useOneX></authEncryption>
 <sharedKey><keyType>passPhrase</keyType><protected>false</protected>
 <keyMaterial>fpga-car-2026</keyMaterial></sharedKey></security></MSM>
</WLANProfile>''', encoding='utf-8')
    added = False
    switched = False
    summary = {'ok': False, 'restored': False, 'packets': 0}
    try:
        netsh('add', 'profile', f'filename={xml}', 'user=current')
        added = True
        # Mark before issuing connect so even a command timeout triggers restoration.
        switched = True
        print('Temporarily joining FPGA-CAR; previous Wi-Fi will be restored.', flush=True)
        netsh('connect', f'name={profile}', f'interface={args.interface}')
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline:
            try:
                connected = current_profile() == profile
            except RuntimeError:
                connected = False
            if connected:
                try:
                    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                        probe.connect(('192.168.4.1', 4242))
                        if probe.getsockname()[0].startswith('192.168.4.'):
                            break
                except OSError:
                    pass  # The association may complete before DHCP installs an IPv4 route.
            time.sleep(.5)
        else:
            raise TimeoutError('Car AP connection/DHCP timed out')
        records = []
        invalid = 0
        stop_packet = None
        stop_seq = None
        stop_ack = False
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(('192.168.4.1', 4242))
            sock.settimeout(.25)
            deadline = time.monotonic() + 6
            next_hello = 0
            next_stop = 0
            while time.monotonic() < deadline:
                if time.monotonic() >= next_hello:
                    sock.send(b'FCAR?')  # Registration only; never send a motor command.
                    next_hello = time.monotonic() + 2
                if stop_packet and not stop_ack and time.monotonic() >= next_stop:
                    sock.send(stop_packet)
                    next_stop = time.monotonic() + .2
                try:
                    raw = sock.recv(2048)
                except (socket.timeout, ConnectionResetError):
                    continue
                try:
                    record = decode(raw, config)
                except ValueError:
                    invalid += 1
                    continue
                if record['enabled'] != bool(args.wheel_probe or args.expect_enabled):
                    raise RuntimeError('Board motor permission differs from the requested test')
                if args.boot_id is not None and record['boot_id'] != args.boot_id:
                    raise RuntimeError('Test boot ID differs from the expected board session')
                records.append(record)
                if args.stop_probe:
                    if not record['boot_id']:
                        raise RuntimeError('Board has not been configured')
                    if stop_seq is None:
                        stop_seq = (record['last_command_seq'] + 1) & 0xffffffff
                        stop_packet = command(stop_seq, record['boot_id'], record['timestamp_ms'], 0)
                    elif record['last_command_seq'] == stop_seq and record['command_status'] == 0:
                        stop_ack = True
        if args.wheel_probe:
            class CapturingClient(Client):
                def receive(self, *values, **kwargs):
                    record = super().receive(*values, **kwargs)
                    records.append(record)
                    baseline = getattr(self, 'pin_check_baseline', None)
                    if args.pwm_pin_check and baseline is not None:
                        if record['boot_id'] != args.boot_id or any(
                                abs(record['encoder_' + side + '_count'] - baseline['encoder_' + side + '_count']) > 2
                                for side in ('left', 'right')):
                            raise RuntimeError('Unexpected movement/restart during motor-power-disconnected pin check')
                        if record['last_action'] == 'wheel_velocity' and not getattr(self, 'pin_check_announced', False):
                            print('PWM_PIN_CHECK_ACTIVE: observe the fixed AIN1/GND meter reading; motor battery must remain disconnected.', flush=True)
                            self.pin_check_announced = True
                    return record
            client = CapturingClient('192.168.4.1', config)
            try:
                before = client.receive()
                if before['boot_id'] != args.boot_id or not before['enabled']:
                    raise RuntimeError('Armed board changed before the wheel test')
                if args.pwm_pin_check:
                    client.pin_check_baseline = before
                left = rpm_target(args.probe_rpm, config, 'left') if args.wheel_probe == 'left' else 0
                right = rpm_target(args.probe_rpm, config, 'right') if args.wheel_probe == 'right' else 0
                client.hold(1, left, right, args.probe_seconds)
                settle_end = time.monotonic() + 1
                while time.monotonic() < settle_end:
                    after = client.receive()
                if after['boot_id'] != args.boot_id or after['last_action'] != 'stop':
                    raise RuntimeError('Wheel test did not finish in STOP on the same board session')
                summary['wheel_probe'] = {
                    'side': args.wheel_probe, 'rpm': args.probe_rpm, 'duration_s': args.probe_seconds,
                    'left_delta': after['encoder_left_count'] - before['encoder_left_count'],
                    'right_delta': after['encoder_right_count'] - before['encoder_right_count'],
                    'last_action': after['last_action'], 'stop_reason': after['stop_reason'],
                    'max_abs_rpm_left': max(abs(x['wheel_rpm_left']) for x in records),
                    'max_abs_rpm_right': max(abs(x['wheel_rpm_right']) for x in records)}
                tested_delta = summary['wheel_probe'][args.wheel_probe + '_delta']
                other_side = 'right' if args.wheel_probe == 'left' else 'left'
                summary['wheel_probe']['motion_observed'] = tested_delta != 0
                summary['wheel_probe']['forward_observed'] = tested_delta > 0
                summary['wheel_probe']['other_wheel_still'] = summary['wheel_probe'][other_side + '_delta'] == 0
            finally:
                try:
                    client.send(0)
                finally:
                    client.close()
        (output / (report_prefix + '_telemetry.jsonl')).write_text(
            ''.join(json.dumps(x, allow_nan=False) + '\n' for x in records), encoding='utf-8')
        summary.update(packets=len(records), invalid_frames=invalid)
        if len(records) < 5:
            raise RuntimeError(f'Only {len(records)} telemetry packets received; need at least 5')
        if len({x['boot_id'] for x in records}) != 1:
            raise RuntimeError('FPGA restarted during the test')
        if records[-1]['timestamp_ms'] == records[0]['timestamp_ms']:
            raise RuntimeError('Telemetry timestamp did not advance')
        summary.update(stop_probe_requested=args.stop_probe, stop_acknowledged=stop_ack)
        if args.stop_probe and not stop_ack:
            raise RuntimeError('STOP was not acknowledged by the FPGA')
        all_actions_stop=all(x['last_action']=='stop' for x in records)
        if args.expect_enabled and not all_actions_stop:
            raise RuntimeError('An unexpected active action was observed during the STOP-only check')
        summary.update(ok=True, first_seq=records[0]['seq'], last_seq=records[-1]['seq'],
                       first_timestamp_ms=records[0]['timestamp_ms'],
                       last_timestamp_ms=records[-1]['timestamp_ms'],
                       all_motors_disabled=all(not x['enabled'] for x in records),
                       all_actions_stop=all_actions_stop,
                       expected_permission=bool(args.wheel_probe or args.expect_enabled),
                       last_errors=records[-1]['errors'])
        if args.pwm_pin_check:
            still = summary['wheel_probe']['left_delta'] == 0 and summary['wheel_probe']['right_delta'] == 0
            summary.update(ok=still, pwm_pin_check=True, expected_no_motion=True,
                           physical_pwm_voltage_v=None)
        elif args.wheel_probe and not (summary['wheel_probe']['forward_observed'] and
                                    summary['wheel_probe']['other_wheel_still']):
            summary.update(ok=False, error='Command/STOP acknowledged, but expected single-wheel forward motion was not observed')
    except Exception as exc:
        summary['error'] = str(exc)
    finally:
        if switched:
            for _ in range(3):
                try:
                    netsh('connect', f'name={args.restore_profile}', f'interface={args.interface}')
                    for _ in range(20):
                        time.sleep(.5)
                        try:
                            restored = current_profile() == args.restore_profile
                        except RuntimeError:
                            restored = False
                        if restored:
                            summary['restored'] = True
                            break
                except (RuntimeError, subprocess.TimeoutExpired):
                    pass
                if summary['restored']:
                    break
        else:
            summary['restored'] = True
        if added:
            try:
                netsh('delete', 'profile', f'name={profile}', f'interface={args.interface}')
            except Exception as exc:
                summary['profile_cleanup_error'] = str(exc)
        xml.unlink(missing_ok=True)
        summary_name = report_prefix + '_summary.json' if args.wheel_probe else 'wifi_test_summary.json'
        (output / summary_name).write_text(
            json.dumps(summary, indent=2) + '\n', encoding='utf-8')
        print(json.dumps(summary), flush=True)
    return 0 if summary['ok'] and summary['restored'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
