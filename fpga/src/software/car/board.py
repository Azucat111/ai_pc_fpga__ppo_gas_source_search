"""PYNQ boot/configuration service. No software motor control loop."""
import argparse
import json
import math
import secrets
import struct
import time
from pathlib import Path
from .config import load_config,registers

class PLBus:
    def __init__(self,mmio):self.mmio=mmio
    def transfer(self,address,register,write=b'',read=0,timeout=1.0):
        if not 0<=address<128 or not 0<=register<256 or len(write)>32 or not 0<=read<=32:
            raise ValueError('Invalid PL I2C transaction')
        if self.mmio.read(0x110)&1: raise RuntimeError('PL I2C mailbox is already busy')
        self.mmio.write(0x104,address);self.mmio.write(0x108,register)
        self.mmio.write(0x10c,len(write)|(read<<8))
        data=write.ljust(32,b'\0')
        for i in range(8):self.mmio.write(0x120+4*i,struct.unpack_from('<I',data,4*i)[0])
        self.mmio.write(0x100,1)
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            status=self.mmio.read(0x110)
            if status&2 and not status&1:
                if status&4: raise OSError(f'I2C transaction failed: 0x{address:02x}/0x{register:02x}')
                return b''.join(struct.pack('<I',self.mmio.read(0x140+4*i)) for i in range(8))[:read]
            time.sleep(0.0005)
        raise TimeoutError('PL I2C mailbox timeout')

def configure(mmio,c,enable=False):
    if mmio.read(0)!=0x43415231: raise RuntimeError('Wrong FPGA bitstream: CAR1 signature missing')
    if mmio.read(4)!=0x00010000: raise RuntimeError('Unsupported FPGA register version')
    mmio.write(0x14,0)
    time.sleep(0.02)
    if enable and (not c['wiring_confirmed'] or c['m1_is_left'] is None):
        raise ValueError('Set m1_is_left and confirm encoder/motor directions before --enable')
    for address,value in registers(c).items():mmio.write(address,value)
    boot_id=secrets.randbits(32) or 1
    mmio.write(0x10,boot_id)
    # Configuration enables permission only; actual motion needs a fresh UART command.
    return boot_id

def snapshot(mmio):
    for _ in range(8):
        seq=mmio.read(0x1f0)
        words=[mmio.read(0x200+4*i) for i in range(45)]
        if mmio.read(0x1f0)==seq:return seq,words
    raise RuntimeError('Could not acquire a coherent snapshot')

def main():
    p=argparse.ArgumentParser(description='Configure PYNQ car and run BME688 compensation')
    p.add_argument('--bit',default='build/output/car.bit')
    p.add_argument('--config',default='config/car.json')
    p.add_argument('--enable',action='store_true',help='Allow motion after direction checks')
    p.add_argument('--bme-mode',choices=('single','gas10'),default='single')
    p.add_argument('--no-bme',action='store_true',help='Bring-up only; BME fields stay invalid')
    p.add_argument('--status-only',action='store_true',help='Read the already loaded core without changing it')
    p.add_argument('--reuse-overlay',action='store_true',help='Configure an already loaded, disabled car core without reloading FPGA')
    p.add_argument('--seconds',type=float,help='Optional finite service duration; exit with motor permission cleared')
    args=p.parse_args()
    if args.seconds is not None and (not math.isfinite(args.seconds) or not 0<args.seconds<=3600):
        p.error('--seconds must be greater than 0 and at most 3600')
    from pynq import Overlay,MMIO
    if args.status_only:
        mmio=MMIO(0x43c00000,0x1000)
        print(json.dumps({'id':hex(mmio.read(0)),'snapshot':snapshot(mmio)}));return
    c=load_config(args.config)
    if args.reuse_overlay:
        from pynq import Clocks
        mmio=MMIO(0x43c00000,0x1000)
        if mmio.read(0)!=0x43415231 or abs(Clocks.fclk0_mhz-5)>.01:
            raise RuntimeError('Expected loaded car core at 5MHz')
        if mmio.read(0x14)&1:
            raise RuntimeError('Stop and disable the existing controller before reusing overlay')
    else:
        overlay=Overlay(str(Path(args.bit).resolve()))
        if 'car_control' not in overlay.ip_dict: raise RuntimeError('car_control not found in HWH')
        info=overlay.ip_dict['car_control'];mmio=MMIO(info['phys_addr'],info['addr_range'])
    if c['imu_mount_confirmed'] and mmio.read(0xcc)!=0x494d5532:
        raise RuntimeError('Confirmed IMU heading requires the v0.1.6 plausibility core')
    if mmio.read(0xd0)!=0x47595231:
        raise RuntimeError('Requires the v0.1.7 gyro straight-heading core')
    if mmio.read(0xd4)!=0x47595232:
        raise RuntimeError('Requires the v0.1.8 gyro relative-turn core')
    boot_id=configure(mmio,c,args.enable)
    bus=PLBus(mmio)
    try:
        version=bus.transfer(0x23,1,read=3)
        bus.transfer(0x23,0x61,bytes([c['imu_algorithm']]))
        print('IMU firmware:', '.'.join(str(x) for x in version),flush=True)
        time.sleep(1)
    except (OSError,TimeoutError) as exc:
        print('IMU initialization failed; heading modes remain unavailable:',exc,flush=True)
    bme=None
    if not args.no_bme:
        try:
            from .bme import BME688
            bme=BME688(bus)
        except (OSError,RuntimeError) as exc:
            print('BME688 unavailable:',exc,'Run tools/build_bme.sh on the PYNQ board.',flush=True)
    scanner=None
    if args.bme_mode=='gas10':
        if bme is None:raise RuntimeError('GAS10 requires a working BME688 service')
        from .gas10_board import Scanner
        scanner=Scanner(bme,mmio)
    mmio.write(0x98,0)
    mmio.write(0x14,int(args.enable))
    print(f'boot_id={boot_id} motion_enabled={args.enable}',flush=True)
    deadline=time.monotonic()+args.seconds if args.seconds is not None else None
    try:
        while deadline is None or time.monotonic()<deadline:
            start=time.monotonic()
            if scanner:
                complete=scanner.poll()
                if complete:print('GAS10:',json.dumps(complete),flush=True)
            elif bme:
                try:
                    values,flags=bme.sample()
                    for i,value in enumerate(values):mmio.write(0x80+4*i,int(value)&0xffffffff)
                    mmio.write(0x98,flags)
                except (OSError,RuntimeError,ValueError) as exc:
                    mmio.write(0x98,0);print('BME sample invalid:',exc,flush=True)
            time.sleep(.005 if scanner else max(0.01,1-(time.monotonic()-start)))
    except KeyboardInterrupt:pass
    finally:
        mmio.write(0x14,0);mmio.write(0x98,0)
        if scanner:mmio.write(0x3c0,0)

if __name__=='__main__':main()
