"""Read known sensor registers through PL; never reload FPGA or enable motors."""
import json
import argparse
import time
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scan',action='store_true',help='Read one byte at register 0x01 of non-reserved I2C addresses; no configuration payload writes')
    args=parser.parse_args()
    from pynq import MMIO, Clocks
    from car.board import PLBus
    from car.debug import read_diagnostics
    mmio=MMIO(0x43c00000,0x1000)
    before=read_diagnostics(mmio)
    if before['motor_permission'] or abs(Clocks.fclk0_mhz-5)>.01:
        raise RuntimeError('Requires disabled car core at 5MHz')
    bus=PLBus(mmio)
    records=[]
    # Zero-length data writes below only select a read register pointer.
    probes=((0x23,0x01,0,'imu_version_pointer'),
            (0x23,0x01,3,'imu_version'),
            (0x23,0x0a,6,'imu_gyro'),
            (0x23,0x26,12,'imu_euler'),
            (0x48,0x01,2,'ads_config'),
            (0x77,0xd0,1,'bme_chip_id'))
    if args.scan:
        probes=tuple((address,0x01,1,'scan_register_01') for address in range(0x08,0x78))
    for repeat in range(1 if args.scan else 3):
        for address,register,length,name in probes:
            if mmio.read(0x14)&1:
                raise RuntimeError('Another controller enabled motors')
            started=time.monotonic()
            row=dict(repeat=repeat,name=name,address=hex(address),register=hex(register),read_length=length)
            try:
                row.update(ok=True,data=bus.transfer(address,register,read=length).hex())
            except (OSError,TimeoutError) as exc:
                row.update(ok=False,error=str(exc))
            row['duration_ms']=round((time.monotonic()-started)*1000,2)
            records.append(row)
            if not args.scan or row['ok']:
                print(json.dumps(row),flush=True)
            time.sleep(.002 if args.scan else .05)
    final=read_diagnostics(mmio)
    result=dict(boot_id=before['boot_id'],probes=records,final=final)
    out=Path('reports')/(('i2c_scan_' if args.scan else 'i2c_known_devices_')+str(before['boot_id'])+'.json')
    out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(report=str(out),motor_permission=final['motor_permission'],
                          responding_addresses=sorted(set(r['address'] for r in records if r['ok'])))),flush=True)


if __name__=='__main__':
    main()
