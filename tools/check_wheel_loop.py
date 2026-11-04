"""Short serial wheel PI trial with encoder-based aborts and measured speed reporting."""
import argparse
import json
import math
from pathlib import Path
import statistics
import sys
import time

from check_car_uart import SerialSocket, ROOT
from car.config import load_config, rpm_target
from car.host import Client
from car.protocol import signed32
from trial_transport import open_transport


def check_feedback(item, baseline, side, rpm, elapsed, check_other=True):
    if side == 'both':
        for selected in ('left', 'right'):
            check_feedback(item, baseline, selected, rpm, elapsed, check_other=False)
        return
    if item['boot_id'] != baseline['boot_id'] or not item['enabled']:
        raise RuntimeError('Board session changed or motor permission was removed')
    other = 'left' if side == 'right' else 'right'
    delta = item['encoder_' + side + '_count'] - baseline['encoder_' + side + '_count']
    if delta * (1 if rpm > 0 else -1) < -8:
        raise RuntimeError('Encoder feedback is opposite to the commanded direction')
    if check_other and abs(item['encoder_' + other + '_count'] - baseline['encoder_' + other + '_count']) > 8:
        raise RuntimeError('Unselected wheel moved')
    if abs(item['wheel_rpm_' + side]) > max(60, abs(rpm) * 2):
        raise RuntimeError('Wheel exceeded trial speed guard')
    if any(item['errors'][k] > baseline['errors'][k] + 3 for k in ('encoder_m1', 'encoder_m2')):
        raise RuntimeError('Encoder invalid transitions increased')
    if elapsed > 2 and abs(delta) < 4:
        raise RuntimeError('No encoder movement within two seconds; stopping stalled trial')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    link = p.add_mutually_exclusive_group(required=True)
    link.add_argument('--port', help='USB-TTL serial port')
    link.add_argument('--ip', help='ESP32 UDP address; connect to its Wi-Fi first')
    p.add_argument('--boot-id', type=int, required=True)
    p.add_argument('--side', choices=('left', 'right', 'both'), required=True)
    p.add_argument('--wheels-raised', action='store_true', required=True)
    p.add_argument('--rpm', type=float, default=20)
    p.add_argument('--seconds', type=float, default=3)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--stop-file', type=Path)
    p.add_argument('--lease-test', action='store_true', help='After the run, omit heartbeats and verify PL timeout parking')
    p.add_argument('--check-imu-stability', action='store_true', help='Require valid IMU and at most 5deg yaw change while body remains fixed')
    a = p.parse_args()
    if not math.isfinite(a.rpm) or not 5 <= abs(a.rpm) <= 40:
        p.error('--rpm magnitude must be 5..40')
    if not math.isfinite(a.seconds) or not 2.5 <= a.seconds <= 8:
        p.error('--seconds must be 2.5..8')
    if a.output.exists() or (a.stop_file and a.stop_file.exists()):
        p.error('Use fresh output and stop-file paths')
    c = load_config(ROOT / 'config/car.json')
    records = []
    baseline = None
    active_since = None
    class CapturingClient(Client):
        def receive(self, *args, **kwargs):
            nonlocal active_since
            item = super().receive(*args, **kwargs)
            records.append(item)
            if a.stop_file and a.stop_file.exists():
                raise KeyboardInterrupt
            if baseline is not None and item['last_action'] == 'wheel_velocity':
                if active_since is None:
                    active_since = time.monotonic()
                    print(json.dumps(dict(active=True, side=a.side, rpm=a.rpm, seconds=a.seconds)), flush=True)
                check_feedback(item, baseline, a.side, a.rpm, time.monotonic() - active_since)
            return item
    transport, address, service_port = open_transport(port=a.port, ip=a.ip)
    client = CapturingClient(address, c, port=service_port, sock=transport)
    summary = dict(ok=False, boot_id=a.boot_id, side=a.side, target_rpm=a.rpm,
                   requested_seconds=a.seconds, transport='udp' if a.ip else 'serial')
    try:
        first = client.receive(timeout=5)
        if first['boot_id'] != a.boot_id or not first['enabled']:
            raise RuntimeError('Wrong or unarmed board session')
        baseline = client.send(0)
        idle_end = time.monotonic() + 1
        while time.monotonic() < idle_end:
            baseline = client.receive()
        selected = ('left', 'right') if a.side == 'both' else (a.side,)
        left = rpm_target(a.rpm, c, 'left') if 'left' in selected else 0
        right = rpm_target(a.rpm, c, 'right') if 'right' in selected else 0
        if a.lease_test:
            started = client.send(1, left, right)
            end = time.monotonic() + a.seconds
            next_heartbeat = time.monotonic() + .1
            last_heartbeat = started
            while time.monotonic() < end:
                item = client.receive(timeout=.4)
                if item['last_action'] != 'wheel_velocity' or item['action_id'] != started['action_id']:
                    raise RuntimeError('Motion was stopped or replaced during lease test')
                if time.monotonic() >= next_heartbeat:
                    last_heartbeat = client.send(4, signed32(started['action_id']))
                    next_heartbeat = time.monotonic() + .1
            print(json.dumps(dict(heartbeats_paused=True)), flush=True)
            deadline = time.monotonic() + .8
            while time.monotonic() < deadline:
                timeout = client.receive(timeout=.3)
                if timeout['last_action'] == 'stop':
                    break
            else:
                raise RuntimeError('PL did not stop after heartbeats were omitted')
            delay = (timeout['timestamp_ms'] - last_heartbeat['timestamp_ms']) & 0xffffffff
            if timeout['stop_reason'] != 'command_timeout' or not 100 <= delay <= 650:
                raise RuntimeError('Unexpected timeout parking reason or delay')
            summary.update(lease_timeout_observed=True, timeout_from_last_heartbeat_ms=delay)
        else:
            client.hold(1, left, right, a.seconds)
        settle_end = time.monotonic() + (2 if a.check_imu_stability else .7)
        while time.monotonic() < settle_end:
            last = client.receive()
        active = [r for r in records if r['last_action'] == 'wheel_velocity']
        if not active:
            raise RuntimeError('No active telemetry captured')
        end_ms = active[-1]['timestamp_ms']
        wheels = {}
        for side in selected:
            delta = last['encoder_' + side + '_count'] - baseline['encoder_' + side + '_count']
            speeds = [r['wheel_rpm_' + side] for r in active if end_ms - r['timestamp_ms'] <= 1000]
            mean = statistics.mean(speeds)
            wheels[side] = dict(encoder_delta=delta, last_second_mean_rpm=mean,
                               last_second_std_rpm=statistics.pstdev(speeds),
                               max_abs_rpm=max(abs(r['wheel_rpm_' + side]) for r in records),
                               feedback_present=delta * a.rpm > 0,
                               speed_converged=abs(mean - a.rpm) <= max(3, abs(a.rpm) * .15))
        if a.side != 'both':
            summary.update(wheels[a.side])
        summary.update(wheels=wheels,
                       ok=all(w['feedback_present'] and w['speed_converged'] for w in wheels.values()),
                       last_action=last['last_action'], last_errors=last['errors'])
        if a.check_imu_stability:
            before = [r for r in records if r['timestamp_ms'] <= baseline['timestamp_ms'] and r['imu_yaw'] is not None]
            measured = [r for r in records if r['timestamp_ms'] > baseline['timestamp_ms']]
            good = [r for r in measured if r['imu_yaw'] is not None]
            yaw0 = statistics.median(r['imu_yaw'] for r in before) if before else None
            drift = max((abs(math.atan2(math.sin(r['imu_yaw']-yaw0),math.cos(r['imu_yaw']-yaw0)))
                         for r in good), default=math.inf) if yaw0 is not None else math.inf
            stable = bool(measured) and len(good)==len(measured) and drift <= math.radians(5)
            summary.update(imu_stable=stable, imu_valid_samples=len(good), imu_samples=len(measured),
                           imu_max_yaw_change_deg=math.degrees(drift) if math.isfinite(drift) else None,
                           imu_max_age_ms=max(r['imu_age_ms'] for r in measured),
                           imu_max_abs_gyro=max(abs(r['imu_gyro_z']) for r in good) if good else None)
            summary['ok'] = summary['ok'] and stable
    except (OSError, RuntimeError, TimeoutError, KeyboardInterrupt) as exc:
        summary['error'] = type(exc).__name__ + ': ' + str(exc)
    finally:
        try:
            stop = client.send(0)
            summary['stop_acknowledged'] = stop['last_action'] == 'stop'
        except (OSError, RuntimeError, TimeoutError, KeyboardInterrupt) as exc:
            summary['stop_error'] = str(exc)
        finally:
            client.close()
        summary.update(frames=transport.frames, crc_errors=transport.crc_errors,
                       discarded_bytes=transport.discarded)
        a.output.mkdir(parents=True, exist_ok=False)
        (a.output / 'telemetry.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in records), encoding='utf-8')
        (a.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
        print(json.dumps(summary), flush=True)
    return 0 if summary['ok'] and summary.get('stop_acknowledged') else 1


if __name__ == '__main__':
    sys.exit(main())
