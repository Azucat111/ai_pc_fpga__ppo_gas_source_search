"""Bounded ground turn using the PL relative-angle controller; operator must clear space."""
import argparse
import json
import math
from pathlib import Path
import time

from check_car_uart import ROOT
from check_ground_straight import yaw_difference
from car.config import load_config
from car.host import Client
from trial_transport import open_transport


def wheel_distances(item,baseline,config):
    return {side:(item['encoder_'+side+'_count']-baseline['encoder_'+side+'_count'])*math.pi*config['wheel_diameter_'+side+'_m']/config['counts_per_revolution_'+side]
            for side in ('left','right')}


def check_turn_feedback(item,baseline,progress,degrees,elapsed,config,gyro_progress=None):
    if item['boot_id']!=baseline['boot_id'] or not item['enabled']:
        raise RuntimeError('Turn session changed or permission was removed')
    if item['imu_yaw'] is None or item['imu_age_ms']>150 or not item['geometry_valid'] or not item['imu_frame_confirmed']:
        raise RuntimeError('Turn lost fresh confirmed IMU data or geometry')
    direction=1 if degrees>0 else -1
    for side,sign in (('left',-direction),('right',direction)):
        delta=item['encoder_'+side+'_count']-baseline['encoder_'+side+'_count']
        if delta*sign < -8:raise RuntimeError('Wheel feedback initially opposite to the requested turn')
        if abs(item['wheel_rpm_'+side])>60:raise RuntimeError('Turn exceeded wheel speed guard')
        if elapsed>2.5 and abs(delta)<4:raise RuntimeError('Turn wheel did not start within 2.5 seconds')
    if any(item['errors'][k]-baseline['errors'][k]>3 for k in ('encoder_m1','encoder_m2')):
        raise RuntimeError('Turn encoder invalid transitions increased')
    if not baseline.get('turn_gyro_heading',False) and progress*direction < -math.radians(8):raise RuntimeError('Body turned in the wrong direction')
    if not baseline.get('turn_gyro_heading',False) and abs(progress)>math.radians(abs(degrees)+30):raise RuntimeError('Body exceeded the turn angle guard')
    if gyro_progress is not None:
        if gyro_progress*direction < -math.radians(8):raise RuntimeError('Gyro reported body turning in the wrong direction')
        if abs(gyro_progress)>math.radians(abs(degrees)+15):raise RuntimeError('Gyro body angle exceeded target plus 15 degrees')
    if any(abs(d)>.25 for d in wheel_distances(item,baseline,config).values()):
        raise RuntimeError('Turn exceeded 0.25m individual wheel displacement guard')
    if abs(item['travelled_distance_m']-baseline['travelled_distance_m'])>.25:
        raise RuntimeError('Turn exceeded 0.25m centre path guard')
    if abs(item['distance_m']-baseline['distance_m'])>.12:
        raise RuntimeError('Turn translated more than 0.12m by wheel odometry')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ip',required=True);p.add_argument('--boot-id',type=int,required=True)
    p.add_argument('--ground-clear',action='store_true',required=True)
    p.add_argument('--degrees',type=float,required=True)
    p.add_argument('--seconds',type=float,default=10)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--stop-file',type=Path)
    a=p.parse_args()
    if not math.isfinite(a.degrees) or not 5<=abs(a.degrees)<=90:p.error('Turn magnitude must be 5..90 degrees')
    if not math.isfinite(a.seconds) or not 3<=a.seconds<=12:p.error('Turn timeout must be 3..12 seconds')
    if a.output.exists() or (a.stop_file and a.stop_file.exists()):p.error('Use fresh output/stop-file paths')
    c=load_config(ROOT/'config/car.json');records=[]
    baseline=None;previous_yaw=None;progress=0.;active_since=None;previous_sample=None;gyro_progress=0.
    class CapturingClient(Client):
        acknowledging_stop=False
        def send(self,mode,*args,**kwargs):
            self.acknowledging_stop=mode==0
            try:return super().send(mode,*args,**kwargs)
            finally:self.acknowledging_stop=False

        def receive(self,*args,**kwargs):
            nonlocal previous_yaw,progress,active_since,previous_sample,gyro_progress
            item=super().receive(*args,**kwargs)
            item['host_received_monotonic_s']=time.monotonic();records.append(item)
            if a.stop_file and a.stop_file.exists():raise KeyboardInterrupt('STOP file requested')
            if baseline is not None and (active_since is not None or item['last_action']=='turn_relative'):
                if active_since is None:
                    active_since=time.monotonic()
                    print(json.dumps(dict(active=True,mode='turn_relative',degrees=a.degrees,timeout_s=a.seconds)),flush=True)
                if item['imu_yaw'] is not None:
                    progress+=yaw_difference(item['imu_yaw'],previous_yaw);previous_yaw=item['imu_yaw']
                if previous_sample is not None:
                    dt=((item['timestamp_ms']-previous_sample['timestamp_ms'])&0xffffffff)/1000
                    if dt<=.5 and all(x['imu_gyro_z'] is not None for x in (item,previous_sample)):
                        gyro_progress+=(item['imu_gyro_z']+previous_sample['imu_gyro_z'])*.5*dt
                    elif not self.acknowledging_stop:raise RuntimeError('Turn lost continuous gyro feedback')
                previous_sample=item
                item['turn_progress_deg']=math.degrees(progress)
                item['gyro_turn_progress_deg']=math.degrees(gyro_progress)
                if not self.acknowledging_stop:
                    check_turn_feedback(item,baseline,progress,a.degrees,time.monotonic()-active_since,c,gyro_progress)
            return item
    transport,address,port=open_transport(ip=a.ip);client=CapturingClient(address,c,port=port,sock=transport)
    summary=dict(ok=False,boot_id=a.boot_id,mode='turn_relative',target_degrees=a.degrees,
                 timeout_seconds=a.seconds,validation_scope='encoder_and_imu_checks_only',physical_movement_confirmed=None)
    try:
        first=client.receive(timeout=5)
        if first['boot_id']!=a.boot_id or not first['enabled']:raise RuntimeError('Wrong or unarmed board session')
        baseline=client.send(0);end=time.monotonic()+1
        while time.monotonic()<end:baseline=client.receive()
        check_turn_feedback(baseline,baseline,0,a.degrees,0,c)
        summary['turn_heading_source']='gyro_relative' if baseline.get('turn_gyro_heading',False) else 'fused_yaw'
        previous_yaw=baseline['imu_yaw']
        previous_sample=baseline
        result=client.hold(3,round(math.radians(a.degrees)*1000),0,a.seconds)
        summary['completed_by_fpga']=result['stop_reason']=='turn_complete'
        end=time.monotonic()+1
        while time.monotonic()<end:last=client.receive()
        measured=[r for r in records if 'turn_progress_deg' in r]
        if not measured:raise RuntimeError('No turn telemetry captured')
        error=math.degrees(progress)-a.degrees
        errors={k:last['errors'][k]-baseline['errors'][k] for k in baseline['errors']}
        summary.update(imu_turn_degrees=math.degrees(progress),final_angle_error_deg=error,
                       gyro_turn_degrees=math.degrees(gyro_progress),gyro_angle_error_deg=math.degrees(gyro_progress)-a.degrees,
                       max_abs_turn_degrees=max(abs(r['turn_progress_deg']) for r in measured),
                       wheel_distance_m=wheel_distances(last,baseline,c),
                       centre_travelled_m=last['travelled_distance_m']-baseline['travelled_distance_m'],
                       centre_displacement_m=last['distance_m']-baseline['distance_m'],
                       imu_max_age_ms=max(r['imu_age_ms'] for r in measured),errors_delta=errors,
                       encoder_delta={s:last['encoder_'+s+'_count']-baseline['encoder_'+s+'_count'] for s in ('left','right')})
        control_error=math.degrees(gyro_progress)-a.degrees if baseline.get('turn_gyro_heading',False) else error
        summary['control_angle_error_deg']=control_error
        summary['ok']=summary['completed_by_fpga'] and abs(control_error)<=5 and abs(math.degrees(gyro_progress)-a.degrees)<=5 and not any(errors[k] for k in ('adc','uart','encoder_m1','encoder_m2'))
    except (OSError,RuntimeError,TimeoutError,KeyboardInterrupt) as e:summary['error']=type(e).__name__+': '+str(e)
    finally:
        try:summary['stop_acknowledged']=client.send(0)['last_action']=='stop'
        except (OSError,RuntimeError,TimeoutError,KeyboardInterrupt) as e:summary['stop_error']=str(e)
        client.close();summary.update(frames=transport.frames,crc_errors=transport.crc_errors)
        summary.setdefault('completed_by_fpga',False)
        if active_since is not None:
            summary.setdefault('imu_turn_degrees',math.degrees(progress))
            summary.setdefault('gyro_turn_degrees',math.degrees(gyro_progress))
            summary.setdefault('final_angle_error_deg',math.degrees(progress)-a.degrees)
            summary.setdefault('gyro_angle_error_deg',math.degrees(gyro_progress)-a.degrees)
            summary.setdefault('control_angle_error_deg',math.degrees(gyro_progress if baseline.get('turn_gyro_heading',False) else progress)-a.degrees)
        summary['ok']=summary['ok'] and transport.crc_errors==0 and summary.get('stop_acknowledged',False)
        a.output.mkdir(parents=True,exist_ok=False)
        (a.output/'telemetry.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records),encoding='utf-8')
        (a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
        (a.output/'commands.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in transport.command_log),encoding='utf-8')
        print(json.dumps(summary),flush=True)
    return 0 if summary['ok'] else 1


if __name__=='__main__':raise SystemExit(main())
