"""Stationary 9/6/9 IMU algorithm comparison; permission stays off, original mode restored."""
import argparse
import json
import math
from pathlib import Path
import statistics
import time


def disabled(mmio):
    if mmio.read(0x14)&1:raise RuntimeError('Motor permission must remain disabled')


def select_algorithm(mmio,bus,mode):
    disabled(mmio);bus.transfer(0x23,0x61,bytes([mode]))
    deadline=time.monotonic()+3
    while time.monotonic()<deadline:
        disabled(mmio)
        if bus.transfer(0x23,0x61,read=1)[0]==mode:return
        time.sleep(.05)
    raise TimeoutError('Algorithm did not read back within 3 seconds')


def collect(mmio,read_sample,seconds):
    rows=[];end=time.monotonic()+seconds;previous_seq=None
    while time.monotonic()<end:
        disabled(mmio);seq,item=read_sample()
        if seq!=previous_seq:
            rows.append(item);previous_seq=seq
            if rows[0]['boot_id']!=item['boot_id']:raise RuntimeError('PL boot changed')
            if item['last_action']!='stop':raise RuntimeError('Another controller requested motion')
            for side in ('left','right'):
                if item['encoder_'+side+'_count']!=rows[0]['encoder_'+side+'_count']:
                    raise RuntimeError('Wheels moved during stationary comparison')
        time.sleep(.02)
    return rows


def summary(rows):
    good=[r for r in rows if r['imu_yaw'] is not None and r['imu_age_ms']<=150]
    angles=[]
    for r in good:
        yaw=r['imu_yaw']
        if not angles:angles.append(yaw)
        else:angles.append(angles[-1]+(yaw-angles[-1]+math.pi)%(2*math.pi)-math.pi)
    gyro=[r['imu_gyro_z'] for r in good]
    return dict(samples=len(rows),valid=len(good),
                yaw_drift_deg=math.degrees(angles[-1]-angles[0]) if angles else None,
                yaw_range_deg=math.degrees(max(angles)-min(angles)) if angles else None,
                gyro_mean_rad_s=statistics.mean(gyro) if gyro else None,
                gyro_std_rad_s=statistics.pstdev(gyro) if gyro else None,
                imu_max_age_ms=max(r['imu_age_ms'] for r in rows),
                errors_delta={k:rows[-1]['errors'][k]-rows[0]['errors'][k] for k in rows[0]['errors']})


def compare(mmio,bus,read_sample,seconds=12,settle_seconds=3):
    disabled(mmio)
    original=bus.transfer(0x23,0x61,read=1)[0]
    if original not in (6,9):raise RuntimeError('Unexpected algorithm register: '+str(original))
    result=dict(ok=False,original_algorithm=original,phases=[],validation_scope='stationary_algorithm_comparison')
    try:
        # Baseline has no sensor writes; compare then return to the same algorithm.
        for label,mode in (('baseline',original),('alternative',6 if original==9 else 9),('restored',original)):
            if label!='baseline':
                select_algorithm(mmio,bus,mode)
            phase=dict(label=label,algorithm=mode);result['phases'].append(phase)
            phase['settling_samples']=collect(mmio,read_sample,settle_seconds)
            phase['samples']=collect(mmio,read_sample,seconds)
            phase['summary']=summary(phase['samples'])
            print(json.dumps(dict(label=label,algorithm=mode,summary=phase['summary'])),flush=True)
        result['ok']=all(p['summary']['samples']>=100 and p['summary']['valid']==p['summary']['samples'] for p in result['phases'])
    except (OSError,RuntimeError,TimeoutError,ValueError,KeyboardInterrupt) as e:
        result['error']=type(e).__name__+': '+str(e)
    finally:
        mmio.write(0x14,0)
        try:
            if bus.transfer(0x23,0x61,read=1)[0]!=original:
                select_algorithm(mmio,bus,original)
            result['restored_algorithm']=bus.transfer(0x23,0x61,read=1)[0]
        except (OSError,RuntimeError,TimeoutError) as e:result['restore_error']=str(e)
        result['motor_permission']=bool(mmio.read(0x14)&1)
        result['ok']=result['ok'] and result.get('restored_algorithm')==original and not result['motor_permission']
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stationary',action='store_true',required=True)
    p.add_argument('--motor-power-disconnected',action='store_true',required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error('Use a fresh report path')
    from pynq import MMIO,Clocks
    from car.board import PLBus,snapshot
    from car.config import load_config
    from car.protocol import decode_snapshot
    from car.debug import read_diagnostics
    mmio=MMIO(0x43c00000,0x1000);before=read_diagnostics(mmio)
    if before['motor_permission'] or not before['imu_plausibility_supported'] or abs(Clocks.fclk0_mhz-5)>.01:
        raise RuntimeError('Requires disabled v0.1.6 core at 5MHz; stop production service first')
    config=load_config('config/car.json')
    def read_sample():
        seq,words=snapshot(mmio);return seq,decode_snapshot(seq,words,config)
    result=compare(mmio,PLBus(mmio),read_sample)
    result.update(before=before,after=read_diagnostics(mmio))
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('phases','before','after')}),flush=True)
    return 0 if result['ok'] else 1


if __name__=='__main__':raise SystemExit(main())
