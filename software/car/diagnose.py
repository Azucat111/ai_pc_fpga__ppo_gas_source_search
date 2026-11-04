"""First board bring-up with motor permission always disabled; no ESP needed."""
import argparse
import json
import math
import time
from pathlib import Path
from .board import PLBus,configure,snapshot
from .config import load_config
from .protocol import decode_snapshot

def main():
    parser=argparse.ArgumentParser(description='Load/test car overlay without allowing motors')
    parser.add_argument('--bit',default='build/output/car.bit')
    parser.add_argument('--config',default='config/car.json')
    parser.add_argument('--seconds',type=float,default=10)
    parser.add_argument('--interval',type=float,default=.5,help='Seconds between snapshots (0.02..1)')
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--monitor-only',action='store_true',help='Read an already loaded, disabled core without reset or sensor writes')
    mode.add_argument('--reuse-overlay',action='store_true',help='Initialize/sample sensors on an already loaded, disabled core without FPGA reload or configuration reset')
    parser.add_argument('--no-bme',action='store_true',help='Skip PS BME initialization during first communication checks')
    args=parser.parse_args()
    if not math.isfinite(args.seconds) or not 0<args.seconds<=3600:
        parser.error('--seconds must be 0..3600')
    if not math.isfinite(args.interval) or not .02<=args.interval<=1:
        parser.error('--interval must be 0.02..1')
    import pynq
    from pynq import Overlay,MMIO,Clocks
    config=load_config(args.config)
    bme=None
    if args.monitor_only or args.reuse_overlay:
        mmio=MMIO(0x43c00000,0x1000)
    else:
        overlay=Overlay(str(Path(args.bit).resolve()))
        if 'car_control' not in overlay.ip_dict:raise RuntimeError('car_control missing in HWH')
        ip=overlay.ip_dict['car_control'];mmio=MMIO(ip['phys_addr'],ip['addr_range'])
    if mmio.read(0)!=0x43415231 or mmio.read(4)!=0x00010000:
        raise RuntimeError('Car overlay signature/version mismatch')
    if (args.monitor_only or args.reuse_overlay) and mmio.read(0x14)&1:
        raise RuntimeError('Motor permission is enabled; stop the controlling service first')
    if args.reuse_overlay and abs(Clocks.fclk0_mhz-5)>.01:
        raise RuntimeError('Expected loaded car overlay at 5MHz')
    try:
        if not args.monitor_only:
            if not args.reuse_overlay:
                configure(mmio,config,enable=False)
            bus=PLBus(mmio)
            try:
                version=bus.transfer(0x23,1,read=3)
                bus.transfer(0x23,0x61,bytes([config['imu_algorithm']]))
                print(json.dumps({'imu_firmware':list(version)}),flush=True)
            except (OSError,TimeoutError) as exc:
                print(json.dumps({'imu_init_error':str(exc)}),flush=True)
            if not args.no_bme:
                try:
                    from .bme import BME688
                    bme=BME688(bus)
                except (OSError,RuntimeError) as exc:
                    print(json.dumps({'bme_init_error':str(exc)}),flush=True)
            mmio.write(0x98,0)
        print(json.dumps({'pynq_version':pynq.__version__,'fclk0_mhz':Clocks.fclk0_mhz,
                          'signature':hex(mmio.read(0)),'motor_permission':mmio.read(0x14)&1}),flush=True)
        end=time.monotonic()+args.seconds;next_bme=0
        while time.monotonic()<end:
            if mmio.read(0x14)&1:raise RuntimeError('Another process enabled motors; diagnostic stopped')
            if bme and time.monotonic()>=next_bme:
                try:
                    values,flags=bme.sample()
                    for i,value in enumerate(values):mmio.write(0x80+4*i,int(value)&0xffffffff)
                    mmio.write(0x98,flags)
                except (OSError,RuntimeError,ValueError) as exc:
                    mmio.write(0x98,0);print(json.dumps({'bme_sample_error':str(exc)}),flush=True)
                next_bme=time.monotonic()+1
            seq,words=snapshot(mmio)
            print(json.dumps(decode_snapshot(seq,words,config),allow_nan=False),flush=True)
            time.sleep(args.interval)
    except KeyboardInterrupt:pass
    finally:
        if not args.monitor_only:
            mmio.write(0x14,0);mmio.write(0x98,0)

if __name__=='__main__':main()
