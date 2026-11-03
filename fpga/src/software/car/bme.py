"""Bosch SensorAPI through the FPGA's arbitrated I2C mailbox."""
import ctypes as C
import math
import time
from pathlib import Path

READ=C.CFUNCTYPE(C.c_int8,C.c_uint8,C.POINTER(C.c_uint8),C.c_uint32,C.c_void_p)
WRITE=READ
DELAY=C.CFUNCTYPE(None,C.c_uint32,C.c_void_p)

class Sample(C.Structure):
    _fields_=[('temperature',C.c_float),('humidity',C.c_float),('pressure',C.c_float),
              ('gas',C.c_float),('status',C.c_uint32)]

class BME688:
    def __init__(self,bus,library=None):
        self.bus=bus;self.last_error=None
        path=Path(library) if library else Path(__file__).with_name('libbme68x_car.so')
        self.lib=C.CDLL(str(path))
        def read(reg,data,length,_context):
            try:
                for offset in range(0,length,32):
                    chunk=bus.transfer(0x77,(reg+offset)&255,read=min(32,length-offset))
                    for i,value in enumerate(chunk):data[offset+i]=value
                return 0
            except Exception as exc:
                self.last_error=exc;return -1
        def write(reg,data,length,_context):
            try:
                # Bosch supplies data, register, data, ... for interleaved writes.
                # Preserve this byte stream; do not reinterpret it as contiguous registers.
                if length>32:raise ValueError('Bosch write exceeds mailbox capacity')
                bus.transfer(0x77,reg,bytes(data[:length]));return 0
            except Exception as exc:
                self.last_error=exc;return -1
        self.callbacks=(READ(read),WRITE(write),DELAY(lambda us,_:time.sleep(us/1e6)))
        self.lib.car_bme_init.argtypes=[READ,WRITE,DELAY];self.lib.car_bme_init.restype=C.c_int
        self.lib.car_bme_sample.argtypes=[C.POINTER(Sample)];self.lib.car_bme_sample.restype=C.c_int
        rc=self.lib.car_bme_init(*self.callbacks)
        if rc:raise RuntimeError(f'Bosch initialization status {rc}: {self.last_error}')

    def sample(self):
        result=Sample();rc=self.lib.car_bme_sample(C.byref(result))
        if rc:raise RuntimeError(f'Bosch sample status {rc}: {self.last_error}')
        vals=[result.temperature,result.humidity,result.pressure,result.gas]
        if not all(math.isfinite(x) for x in vals):raise ValueError('Non-finite BME688 sample')
        if not -50<=vals[0]<=100 or not 0<=vals[1]<=100 or not 10000<=vals[2]<=150000 or not 0<=vals[3]<=0xffffffff:
            raise ValueError('BME688 result outside supported physical range')
        flags=int(bool(result.status&0x80))|(int(bool(result.status&0x20))<<1)|(int(bool(result.status&0x10))<<2)
        return [round(vals[0]*100),round(vals[1]*1000),round(vals[2]),round(vals[3])],flags

    def begin_profile(self,temperature,duration_ms):
        fn=getattr(self.lib,'car_bme_begin_profile',None)
        if fn is None:raise RuntimeError('Rebuild BME library for GAS10 support')
        fn.argtypes=[C.c_uint16,C.c_uint16];fn.restype=C.c_int
        wait_us=fn(temperature,duration_ms)
        if wait_us<0:raise RuntimeError(f'Bosch profile start status {wait_us}: {self.last_error}')
        if not 1000<=wait_us<=2000000:raise RuntimeError('Unexpected BME measurement duration')
        return wait_us/1e6

    def finish_profile(self):
        fn=getattr(self.lib,'car_bme_finish_profile',None)
        if fn is None:raise RuntimeError('Rebuild BME library for GAS10 support')
        fn.argtypes=[C.POINTER(Sample)];fn.restype=C.c_int
        result=Sample();rc=fn(C.byref(result))
        if rc:raise RuntimeError(f'Bosch profile read status {rc}: {self.last_error}')
        values=[result.temperature,result.humidity,result.pressure,result.gas]
        if not all(math.isfinite(x) for x in values):raise ValueError('Non-finite GAS10 point')
        if not -50<=values[0]<=100 or not 0<=values[1]<=100 or not 10000<=values[2]<=150000 or not 0<values[3]<=0xffffffff:
            raise ValueError('GAS10 point outside physical range')
        flags=int(bool(result.status&0x80))|(int(bool(result.status&0x20))<<1)|(int(bool(result.status&0x10))<<2)
        return [round(values[0]*100),round(values[1]*1000),round(values[2]),round(values[3])],flags
