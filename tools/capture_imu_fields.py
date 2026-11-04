"""Stationary sensor-only capture through PL I2C; no sensor configuration writes."""
import argparse
import json
import math
from pathlib import Path
import struct
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stationary',action='store_true',required=True)
    p.add_argument('--power-label',default='operator-state-not-confirmed')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error('Use a fresh output file')
    from pynq import MMIO
    from car.board import PLBus,snapshot
    from car.config import load_config
    from car.debug import read_diagnostics
    from car.protocol import decode_snapshot
    m=MMIO(0x43c00000,0x1000);before=read_diagnostics(m)
    if before['motor_permission']:raise RuntimeError('Stop the production service and disable motion first')
    c=load_config('config/car.json');bus=PLBus(m);rows=[];result=dict(ok=False,validation_scope='sensor_read_capture_only',power_label=a.power_label,before=before)
    try:
        result['algorithm_reads']=[bus.transfer(0x23,0x61,read=1)[0] for _ in range(3)]
        end=time.monotonic()+10
        while time.monotonic()<end:
            if m.read(0x14)&1:raise RuntimeError('Motion permission changed during sensor capture')
            seq,w=snapshot(m);item=decode_snapshot(seq,w,c)
            if item['last_action']!='stop':raise RuntimeError('Unexpected action during stationary capture')
            if rows and any(item['encoder_'+s+'_count']!=rows[0]['telemetry']['encoder_'+s+'_count'] for s in ('left','right')):
                raise RuntimeError('Wheel movement during stationary capture')
            row=dict(telemetry=item);rows.append(row)
            try:
                raw=bus.transfer(0x23,0x26,read=12);row['euler_bytes_hex']=raw.hex()
                euler=list(struct.unpack('<fff',raw))
                if not all(math.isfinite(v) for v in euler):raise ValueError('Non-finite raw Euler data')
                row['euler_rad']=euler
                row['gyro_raw']=list(struct.unpack('<hhh',bus.transfer(0x23,0x0a,read=6)))
                row['mag_raw']=list(struct.unpack('<hhh',bus.transfer(0x23,0x10,read=6)))
                row['mag_uT']=[v*800/32767 for v in row['mag_raw']]
                row['mag_magnitude_uT']=math.sqrt(sum(v*v for v in row['mag_uT']))
            except (OSError,TimeoutError,ValueError) as e:row['read_error']=str(e)
            time.sleep(.1)
        result['ok']=len(rows)>=30 and all('read_error' not in r for r in rows)
    except (OSError,RuntimeError,TimeoutError,KeyboardInterrupt) as e:result['error']=type(e).__name__+': '+str(e)
    finally:
        m.write(0x14,0);result.update(rows=rows,after=read_diagnostics(m))
        a.output.parent.mkdir(parents=True,exist_ok=True)
        a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        print(json.dumps(dict(ok=result['ok'],samples=len(rows),algorithm_reads=result.get('algorithm_reads'),power_label=a.power_label,output=str(a.output))),flush=True)
    return 0 if result['ok'] else 1


if __name__=='__main__':raise SystemExit(main())
