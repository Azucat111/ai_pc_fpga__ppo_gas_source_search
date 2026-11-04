"""Read Yahboom IMU UART version/attitude at 115200; sends data requests only."""
import argparse
import json
import math
from pathlib import Path
import struct
import time


def request(function):
    if function not in (0x01,0x04,0x26):
        raise ValueError('Only read-data requests are permitted')
    data=bytes((0x7e,0x23,7,0x80,function,0))
    return data+bytes((sum(data)&255,))


def take_frames(buffer):
    frames=[]
    bad=0
    while True:
        i=buffer.find(b'\x7e\x23')
        if i<0:
            if len(buffer)>1:
                del buffer[:-1]
            break
        del buffer[:i]
        if len(buffer)<3:
            break
        length=buffer[2]
        if not 5<=length<=40:
            del buffer[0]
            bad+=1
            continue
        if len(buffer)<length:
            break
        frame=bytes(buffer[:length])
        if sum(frame[:-1])&255!=frame[-1]:
            del buffer[0]
            bad+=1
            continue
        del buffer[:length]
        frames.append((frame[3],frame[4:-1]))
    return frames,bad


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',required=True)
    p.add_argument('--seconds',type=float,default=5)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if not math.isfinite(a.seconds) or not 2<=a.seconds<=15:
        p.error('--seconds must be 2..15')
    if a.output.exists():
        p.error('Use a fresh output directory')
    import serial
    raw=bytearray();buffer=bytearray();records=[];bad=0;counts={};version=None
    with serial.Serial(a.port,115200,timeout=.02,write_timeout=.5) as link:
        link.dtr=False;link.rts=False
        start=time.monotonic();next_request=0
        while time.monotonic()-start<a.seconds:
            elapsed=time.monotonic()-start
            if elapsed>=next_request:
                link.write(request(0x01 if version is None else 0x04))
                link.write(request(0x26))
                next_request=elapsed+.5
            chunk=link.read(min(4096,max(1,link.in_waiting)))
            raw.extend(chunk);buffer.extend(chunk)
            frames,errors=take_frames(buffer);bad+=errors
            for function,payload in frames:
                key=hex(function);counts[key]=counts.get(key,0)+1
                row=dict(elapsed_s=round(elapsed,3),function=key,payload_hex=payload.hex())
                if function==1 and len(payload)==3:
                    version=list(payload);row['version']=version
                elif function==0x26 and len(payload)==12:
                    values=struct.unpack('<3f',payload)
                    if all(math.isfinite(x) for x in values):
                        row.update(roll=values[0],pitch=values[1],yaw=values[2])
                elif function==4 and len(payload)==18:
                    row['gyro_z_rad_s']=struct.unpack_from('<h',payload,10)[0]*2000/32767*math.pi/180
                records.append(row)
    summary=dict(port=a.port,baud=115200,ok=bool(records),received_bytes=len(raw),
                 valid_frames=len(records),checksum_or_length_errors=bad,
                 functions=counts,version=version,
                 last_euler=next((r for r in reversed(records) if 'yaw' in r),None),
                 last_gyro=next((r for r in reversed(records) if 'gyro_z_rad_s' in r),None))
    a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'capture.bin').write_bytes(raw)
    (a.output/'frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records),encoding='utf-8')
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(summary),flush=True)
    return 0 if summary['ok'] else 1


if __name__=='__main__':
    raise SystemExit(main())
