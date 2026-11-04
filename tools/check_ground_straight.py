"""Bounded ground trial using the FPGA straight-heading mode over UART or UDP."""
import argparse
import json
import math
from pathlib import Path
import statistics
import time

from check_car_uart import ROOT, SerialSocket
from check_wheel_loop import check_feedback
from car.config import load_config, rpm_target
from car.host import Client
from trial_transport import open_transport


def yaw_difference(current, initial):
    return math.atan2(math.sin(current-initial), math.cos(current-initial))


class HeadingConsistency:
    """Compare short-run fused yaw with gyro integration; this is a guard, not control."""
    def __init__(self,baseline):
        self.previous=baseline;self.yaw_change=0.;self.gyro_integral=0.
        self.gyro_control=baseline.get('straight_gyro_heading',False)

    def update(self,item):
        previous=self.previous
        dt=((item['timestamp_ms']-previous['timestamp_ms'])&0xffffffff)/1000
        if dt>.5:raise RuntimeError('Heading consistency telemetry gap exceeded 0.5 seconds')
        if any(r[k] is None for r in (previous,item) for k in ('imu_yaw','imu_gyro_z')):
            raise RuntimeError('Heading consistency lost yaw or gyro data')
        self.yaw_change+=yaw_difference(item['imu_yaw'],previous['imu_yaw'])
        self.gyro_integral+=(item['imu_gyro_z']+previous['imu_gyro_z'])*.5*dt
        self.previous=item
        disagreement=self.yaw_change-self.gyro_integral
        item['heading_gyro_integral_deg']=math.degrees(self.gyro_integral)
        item['heading_yaw_change_deg']=math.degrees(self.yaw_change)
        item['heading_yaw_gyro_disagreement_deg']=math.degrees(disagreement)
        if self.gyro_control and abs(self.gyro_integral)>math.radians(15):
            raise RuntimeError('Gyro-relative heading deviated more than 15 degrees')
        if not self.gyro_control and abs(disagreement)>math.radians(6):
            raise RuntimeError('Fused yaw disagreed with integrated gyro by more than 6 degrees')


def straight_timing(records):
    active=[r for r in records if r['last_action']=='straight']
    if not active:return {}
    first,last=active[0],active[-1]
    after=[r for r in records if r['last_action']=='stop'
           and 0<((r['timestamp_ms']-last['timestamp_ms'])&0xffffffff)<0x80000000]
    if not after:return {}
    stopped=after[0]
    onset=next((r for r in active
                if all(r['encoder_'+s+'_count']-first['encoder_'+s+'_count']>=4
                       and r['wheel_rpm_'+s]>2 for s in ('left','right'))),None)
    elapsed=lambda a,b:((a['timestamp_ms']-b['timestamp_ms'])&0xffffffff)/1000
    return dict(command_active_seconds=elapsed(stopped,first),
                encoder_start_delay_seconds=elapsed(onset,first) if onset else None,
                encoder_start_to_stop_seconds=elapsed(stopped,onset) if onset else None,
                observed_stop_reason=stopped['stop_reason'])


def check_ground_feedback(item, baseline, elapsed, rpm, max_distance=.5):
    check_feedback(item, baseline, 'both', rpm, elapsed)
    if (item['imu_yaw'] is None or item['imu_age_ms'] > 150
            or not item['geometry_valid'] or not item['imu_frame_confirmed']):
        raise RuntimeError('Ground trial lost confirmed geometry or fresh IMU data')
    if not item.get('straight_gyro_heading',False) and abs(yaw_difference(item['imu_yaw'], baseline['imu_yaw'])) > math.radians(15):
        raise RuntimeError('Heading deviated more than 15 degrees')
    distance = item['travelled_distance_m']
    if distance is None or abs(distance-baseline['travelled_distance_m']) > max_distance:
        raise RuntimeError('Ground trial exceeded its travel bound')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    link = p.add_mutually_exclusive_group(required=True)
    link.add_argument('--port', help='USB-TTL serial port')
    link.add_argument('--ip', help='ESP32 UDP address; connect to its Wi-Fi first')
    p.add_argument('--boot-id', type=int, required=True)
    p.add_argument('--ground-clear', action='store_true', required=True)
    p.add_argument('--rpm', type=float, default=20)
    p.add_argument('--seconds', type=float, default=3)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--stop-file', type=Path)
    a = p.parse_args()
    if not math.isfinite(a.rpm) or not 10 <= a.rpm <= 20:
        p.error('--rpm must be 10..20 for the first forward ground trial')
    if not math.isfinite(a.seconds) or not 1 <= a.seconds <= 6:
        p.error('--seconds must be 1..6')
    if a.output.exists() or (a.stop_file and a.stop_file.exists()):
        p.error('Use fresh output and stop-file paths')
    c = load_config(ROOT/'config/car.json')
    records = []
    baseline = None
    active_since = None
    consistency = None
    heading_monitor = False

    class CapturingClient(Client):
        def send(self,mode,*args,**kwargs):
            nonlocal heading_monitor
            # A triggered guard must never block receipt of the STOP acknowledgement.
            heading_monitor=mode==2
            return super().send(mode,*args,**kwargs)

        def receive(self, *args, **kwargs):
            nonlocal active_since,consistency
            item = super().receive(*args, **kwargs)
            item['host_received_monotonic_s']=time.monotonic()
            records.append(item)
            if a.stop_file and a.stop_file.exists():
                raise KeyboardInterrupt('Stop file requested')
            if baseline is not None:
                if item['boot_id'] != a.boot_id or not item['enabled']:
                    raise RuntimeError('Ground trial session changed or permission was removed')
                if item['last_action'] == 'straight':
                    if active_since is None:
                        active_since = time.monotonic()
                        consistency = HeadingConsistency(baseline)
                        print(json.dumps(dict(active=True, mode='straight', rpm=a.rpm,
                                              seconds=a.seconds, max_distance_m=.5)), flush=True)
                    check_ground_feedback(item, baseline, time.monotonic()-active_since, a.rpm)
                if consistency is not None and heading_monitor:
                    consistency.update(item)
            return item

    transport, address, service_port = open_transport(port=a.port, ip=a.ip)
    client = CapturingClient(address, c, port=service_port, sock=transport)
    summary = dict(ok=False, boot_id=a.boot_id, mode='straight', rpm=a.rpm,
                   requested_seconds=a.seconds, transport='udp' if a.ip else 'serial',
                   validation_scope='encoder_and_imu_checks_only',physical_movement_confirmed=None)
    try:
        first = client.receive(timeout=5)
        if first['boot_id'] != a.boot_id or not first['enabled']:
            raise RuntimeError('Wrong or unarmed board session')
        baseline = client.send(0)
        idle_end = time.monotonic()+1
        while time.monotonic() < idle_end:
            baseline = client.receive()
        check_ground_feedback(baseline, baseline, 0, a.rpm)
        left = rpm_target(a.rpm, c, 'left')
        right = rpm_target(a.rpm, c, 'right')
        client.hold(2, left, right, a.seconds)
        settle_end = time.monotonic()+1
        while time.monotonic() < settle_end:
            last = client.receive()
            check_ground_feedback(last, baseline, 0, a.rpm)
        active = [r for r in records if r['last_action'] == 'straight']
        if not active:
            raise RuntimeError('No straight-mode telemetry captured')
        measured = [r for r in records if r['timestamp_ms'] > baseline['timestamp_ms']]
        distance = last['travelled_distance_m']-baseline['travelled_distance_m']
        yaw_error = math.degrees(yaw_difference(last['imu_yaw'], baseline['imu_yaw']))
        gyro_control=baseline.get('straight_gyro_heading',False)
        heading_error=math.degrees(consistency.gyro_integral) if gyro_control else yaw_error
        wheels = {}
        for side in ('left', 'right'):
            delta = last['encoder_'+side+'_count']-baseline['encoder_'+side+'_count']
            speeds = [r['wheel_rpm_'+side] for r in active
                      if active[-1]['timestamp_ms']-r['timestamp_ms'] <= 1000]
            wheels[side] = dict(encoder_delta=delta, last_second_mean_rpm=statistics.mean(speeds))
        errors = {k:last['errors'][k]-baseline['errors'][k] for k in baseline['errors']}
        summary.update(travelled_m=distance, final_yaw_error_deg=yaw_error, wheels=wheels,
                       straight_heading_source='gyro_relative' if gyro_control else 'fused_yaw',
                       final_heading_error_deg=heading_error,
                       max_yaw_error_deg=max(abs(math.degrees(yaw_difference(r['imu_yaw'], baseline['imu_yaw'])))
                                             for r in measured),
                       imu_max_age_ms=max(r['imu_age_ms'] for r in measured), errors_delta=errors)
        summary['ok'] = (all(w['encoder_delta'] > 0 for w in wheels.values())
                         and .05 <= distance <= .5 and abs(heading_error) <= 5
                         and not any(errors[k] for k in ('adc', 'uart', 'encoder_m1', 'encoder_m2')))
    except (OSError, RuntimeError, TimeoutError, KeyboardInterrupt) as exc:
        summary['error'] = type(exc).__name__+': '+str(exc)
    finally:
        try:
            stop = client.send(0)
            summary['stop_acknowledged'] = stop['last_action'] == 'stop'
        except (OSError, RuntimeError, TimeoutError, KeyboardInterrupt) as exc:
            summary['stop_error'] = str(exc)
        finally:
            client.close()
        summary.update(frames=transport.frames, crc_errors=transport.crc_errors)
        if consistency is not None:
            summary.update(active_gyro_integral_deg=math.degrees(consistency.gyro_integral),
                           active_yaw_gyro_disagreement_deg=math.degrees(consistency.yaw_change-consistency.gyro_integral))
        summary.update(straight_timing(records))
        summary['ok'] = summary['ok'] and transport.crc_errors == 0 and summary.get('stop_acknowledged', False)
        a.output.mkdir(parents=True, exist_ok=False)
        (a.output/'telemetry.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records), encoding='utf-8')
        (a.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n', encoding='utf-8')
        (a.output/'commands.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in getattr(transport,'command_log',[])),encoding='utf-8')
        print(json.dumps(summary), flush=True)
    return 0 if summary['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
