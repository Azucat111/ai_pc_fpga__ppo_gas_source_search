"""Explicit stationary, level IMU accel/gyro calibration through PL I2C."""
import argparse
import json
import math
from pathlib import Path
import statistics
import struct
import time


def check_setup(rows, accel_vectors):
    if len(rows)<10 or any(r['imu_yaw'] is None or r['imu_age_ms']>150 or r['enabled'] for r in rows):
        raise RuntimeError('Calibration requires continuous fresh IMU data and motor permission off')
    if any(rows[-1]['encoder_'+s+'_count']!=rows[0]['encoder_'+s+'_count'] for s in ('left','right')):
        raise RuntimeError('Wheels moved during calibration preflight')
    gyro=[r['imu_gyro_z'] for r in rows]
    if statistics.pstdev(gyro)>.003:
        raise RuntimeError('Gyro varies during preflight; hold the entire car stationary')
    tilts=[math.degrees(math.atan2(math.hypot(v[0],v[1]),abs(v[2])))
           for v in accel_vectors if any(v)]
    if len(tilts)<3:
        raise RuntimeError('Not enough nonzero acceleration reads to check level')
    if statistics.median(v[2] for v in accel_vectors if any(v))<=0:
        raise RuntimeError('Acceleration indicates the module is not face up')
    tilt=statistics.median(tilts)
    # --level is the operator's independent physical reference. An uncalibrated
    # accelerometer cannot distinguish its own offset from a modest mount tilt.
    return dict(accel_tilt_median_deg=tilt,apparent_tilt_warning=tilt>2,
                level_reference='Operator confirmed physical level; not inferred from IMU',
                gyro_std_rad_s=statistics.pstdev(gyro))


def result_is_usable(summary):
    return (summary['samples']>=100 and summary['valid']==summary['samples']
            and summary['max_age_ms']<=150 and abs(summary['gyro_mean'])<=.003
            and summary['gyro_std']<=.003)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stationary',action='store_true',required=True)
    p.add_argument('--level',action='store_true',required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():
        p.error('Use a fresh output report')
    from pynq import MMIO,Clocks
    from car.board import PLBus,snapshot
    from car.config import load_config
    from car.debug import read_diagnostics
    from car.protocol import decode_snapshot
    m=MMIO(0x43c00000,0x1000)
    before=read_diagnostics(m)
    if before['motor_permission'] or not before['imu_last_good_supported'] or abs(Clocks.fclk0_mhz-5)>.01:
        raise RuntimeError('Requires disabled v0.1.5 core at 5MHz')
    c=load_config('config/car.json');bus=PLBus(m)
    def sample(seconds):
        rows=[];end=time.monotonic()+seconds
        while time.monotonic()<end:
            if m.read(0x14)&1:
                raise RuntimeError('Another controller enabled motors')
            seq,words=snapshot(m);rows.append(decode_snapshot(seq,words,c));time.sleep(.02)
        return rows
    def summary(rows):
        good=[r for r in rows if r['imu_yaw'] is not None]
        gyro=[r['imu_gyro_z'] for r in good]
        return dict(samples=len(rows),valid=len(good),gyro_mean=statistics.mean(gyro),
                    gyro_median=statistics.median(gyro),gyro_std=statistics.pstdev(gyro),
                    max_age_ms=max(r['imu_age_ms'] for r in rows))
    result=dict(boot_id=before['boot_id'],ok=False,calibration_requested=False)
    try:
        result['before_samples']=sample(1)
        result['before_summary']=summary(result['before_samples'])
        vectors=[]
        for _ in range(5):
            if m.read(0x14)&1:raise RuntimeError('Another controller enabled motors')
            vectors.append(struct.unpack('<hhh',bus.transfer(0x23,4,read=6)))
            time.sleep(.02)
        result['accel_preflight_raw']=vectors
        result['preflight']=check_setup(result['before_samples'],vectors)
        print(json.dumps(dict(calibrating=True,before=result['before_summary'])),flush=True)
        # 0x70=1 starts calibration; do not write 0 (clear) or 0xA0 (reset).
        result['calibration_requested']=True
        bus.transfer(0x23,0x70,write=b'\x01')
        started=time.monotonic();states=[]
        while time.monotonic()-started<10:
            time.sleep(.1)
            if m.read(0x14)&1:
                raise RuntimeError('Another controller enabled motors')
            try:
                status=bus.transfer(0x23,0x70,read=1)[0]
                states.append(dict(elapsed_s=round(time.monotonic()-started,3),status=status))
            except (OSError,TimeoutError) as exc:
                states.append(dict(elapsed_s=round(time.monotonic()-started,3),error=str(exc)))
                continue
            if status==1 and time.monotonic()-started>=2:
                break
        else:
            raise TimeoutError('IMU calibration did not report success within 10 seconds')
        result.update(states=states,status=status)
        # Calibration can reinitialize the module's fusion output. Record this
        # disabled settling interval separately; still require five fully fresh
        # seconds afterwards, without changing the FPGA's 150ms motion timeout.
        result['settling_samples']=sample(1)
        result['settling_summary']=summary(result['settling_samples'])
        result['after_samples']=sample(5)
        result['after_summary']=summary(result['after_samples'])
        result['ok']=status==1 and result_is_usable(result['after_summary'])
        if not result['ok']:result['error']='Calibration status/data did not meet static residual limits'
    except (OSError,RuntimeError,TimeoutError,ValueError) as exc:
        result['error']=type(exc).__name__+': '+str(exc)
    finally:
        m.write(0x14,0)
        result['final']=read_diagnostics(m)
        a.output.parent.mkdir(parents=True,exist_ok=True)
        a.output.write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k not in ('before_samples','settling_samples','after_samples','states','final')}),flush=True)
    return 0 if result['ok'] else 1


if __name__=='__main__':
    raise SystemExit(main())
