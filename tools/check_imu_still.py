"""Observe stationary IMU yaw through UDP; sends STOP commands only."""
import argparse
import json
import math
from pathlib import Path
import time

from check_car_uart import ROOT
from check_ground_straight import yaw_difference
from car.config import load_config
from car.host import Client
from trial_transport import UDPTransport


def check_stationary(item,baseline):
    if item['boot_id']!=baseline['boot_id']:raise RuntimeError('Board restarted during observation')
    if item['last_action']!='stop':raise RuntimeError('Another motion action appeared')
    if item['imu_yaw'] is None or item['imu_age_ms']>150:raise RuntimeError('IMU data lost freshness')
    for side in ('left','right'):
        if abs(item['encoder_'+side+'_count']-baseline['encoder_'+side+'_count'])>3 or abs(item['wheel_rpm_'+side])>1:
            raise RuntimeError('Wheel movement detected; keep the car still')


def observe(client,seconds=12):
    records=[];summary=dict(ok=False,validation_scope='stationary_observation_only',requested_seconds=seconds)
    try:
        client.receive(timeout=5);baseline=client.send(0)
        check_stationary(baseline,baseline);records.append(baseline)
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            item=client.receive(timeout=1)
            records.append(item);check_stationary(item,baseline)
        progress=[0.]
        for previous,item in zip(records,records[1:]):
            progress.append(progress[-1]+yaw_difference(item['imu_yaw'],previous['imu_yaw']))
        gyro=[r['imu_gyro_z'] for r in records if r['imu_gyro_z'] is not None]
        summary.update(ok=len(records)>=100,samples=len(records),
                       yaw_drift_deg=math.degrees(progress[-1]),
                       yaw_range_deg=math.degrees(max(progress)-min(progress)),
                       gyro_mean_rad_s=sum(gyro)/len(gyro) if gyro else None,
                       imu_max_age_ms=max(r['imu_age_ms'] for r in records),
                       errors_delta={k:records[-1]['errors'][k]-baseline['errors'][k] for k in baseline['errors']})
    except (OSError,RuntimeError,TimeoutError,KeyboardInterrupt) as e:
        summary['error']=type(e).__name__+': '+str(e)
    finally:
        try:summary['stop_acknowledged']=client.send(0)['last_action']=='stop'
        except (OSError,RuntimeError,TimeoutError,KeyboardInterrupt) as e:summary['stop_error']=str(e)
        summary['ok']=summary['ok'] and summary.get('stop_acknowledged',False)
    return summary,records


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ip',required=True);p.add_argument('--output',required=True,type=Path)
    a=p.parse_args()
    if a.output.exists():p.error('Output must be a fresh directory')
    transport=UDPTransport(a.ip);client=Client(a.ip,load_config(ROOT/'config/car.json'),sock=transport)
    try:summary,records=observe(client)
    finally:client.close()
    summary.update(frames=transport.frames,crc_errors=transport.crc_errors)
    summary['ok']=summary['ok'] and not transport.crc_errors
    a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    (a.output/'telemetry.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records),encoding='utf-8')
    (a.output/'commands.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in transport.command_log),encoding='utf-8')
    print(json.dumps(summary),flush=True)
    return 0 if summary['ok'] else 1


if __name__=='__main__':raise SystemExit(main())
