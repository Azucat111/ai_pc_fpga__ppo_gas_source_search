"""Read-only sensor soak test on an already running car overlay; no I2C calls."""
import argparse
import json
from pathlib import Path
import struct
import time

from car.board import snapshot
from car.config import load_config
from car.debug import read_diagnostics
from car.gas10 import decode_gas10
from car.protocol import crc16,decode_snapshot


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=int,default=180)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not 1<=args.seconds<=600:parser.error('--seconds must be 1..600')
    if args.output.exists():parser.error('Use a new output file')
    from pynq import MMIO
    m=MMIO(0x43c00000,0x1000)
    if m.read(0)!=0x43415231 or m.read(0xdc)!=0x49324332:
        raise RuntimeError('Expected CAR1 with I2C2 recovery/fairness capability')
    config=load_config('config/car.json');boot=m.read(0x10)
    result=dict(scope='read_only_MMIO_not_wireless',boot_id=boot,seconds=args.seconds,
                before=read_diagnostics(m),scans=[],telemetry=[],error=None)
    seen=set();end=time.monotonic()+args.seconds;next_progress=0
    try:
        while time.monotonic()<end:
            if m.read(0x10)!=boot:raise RuntimeError('Controller restarted during validation')
            seq,words=snapshot(m);state=decode_snapshot(seq,words,config)
            result['telemetry'].append(state)
            count=m.read(0x3b8)
            if count and count not in seen:
                words=[m.read(0x400+4*i) for i in range(45)]
                if count==m.read(0x3b8):
                    # This is PL mailbox readback, not an observed UART/Wi-Fi packet.
                    body=b'\xa5\x5a\x01\x82'+struct.pack('<I45I',count,*words)+b'\0\0'
                    scan=decode_gas10(body+crc16(body).to_bytes(2,'little'))
                    seen.add(count);result['scans'].append(scan)
            if state['last_action']!='stop':raise RuntimeError('Expected STOP during sensor-only validation')
            if time.monotonic()>=next_progress:
                print(json.dumps(dict(remaining_s=round(end-time.monotonic()),scans=len(seen),
                                      latest_mask=result['scans'][-1]['gas_valid_mask'] if seen else None,
                                      imu_age_ms=state['imu_age_ms'],ps_status=m.read(0x110))),flush=True)
                next_progress=time.monotonic()+15
            time.sleep(.1)
    except (OSError,RuntimeError,ValueError,KeyboardInterrupt) as exc:
        result['error']=type(exc).__name__+': '+str(exc)
    finally:
        states=result['telemetry'];scans=result['scans']
        result['after']=read_diagnostics(m)
        result['registers']={hex(a):m.read(a) for a in (0xdc,0x110,0x3b4,0x3b8,0x3bc,0x3c0,0x3c4,0x3c8)}
        result['summary']=dict(telemetry_samples=len(states),unique_scans=len(scans),
            valid_scans=sum(s['acquisition_valid'] for s in scans),
            max_imu_age_ms=max((s['imu_age_ms'] for s in states),default=None),
            imu_valid_samples=sum(s['imu_yaw'] is not None for s in states),
            bme_valid_samples=sum(s['bme688']['valid'] for s in states),
            tgs_valid_samples=[sum(s['tgs2602'][i]['valid'] for s in states) for i in (0,1)],
            scan_duration_ms=[s['scan_duration_ms'] for s in scans])
        result['ok']=bool(not result['error'] and len(scans)>=3 and all(s['acquisition_valid'] for s in scans)
                          and states and all(s['imu_yaw'] is not None for s in states)
                          and not any(result['after']['output_logic'].values())
                          and result['before']['output_rising_edges']==result['after']['output_rising_edges'])
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
        print(json.dumps(dict(ok=result['ok'],error=result['error'],summary=result['summary'])),flush=True)
    return 0 if result['ok'] else 1


if __name__=='__main__':raise SystemExit(main())
