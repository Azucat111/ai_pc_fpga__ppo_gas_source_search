"""Single-owner, nonblocking-heating GAS10 acquisition and atomic PS publication."""
import time
import logging
from .gas10 import TEMPERATURES,DURATION_MS,PROFILE_ID,PROFILE_CRC,GAS_SIGNATURE


class Scanner:
    def __init__(self,sensor,mmio,clock=time.monotonic):
        if mmio.read(0xd8)!=GAS_SIGNATURE:raise RuntimeError('GAS1 sensor mailbox missing')
        self.sensor=sensor;self.m=mmio;self.clock=clock;self.scan_id=0;self.index=0
        self.wait_until=None;self.errors=0;self.last_complete=None;self.boot=mmio.read(0x10)
        self.last_error=None;self._next_error_log=0;self.retry_after=0
        self._reset()
        self.m.write(0x3c0,1)

    def _reset(self):
        self.values=[0]*10;self.times=[0]*10;self.status=[0]*10;self.mask=0
        self.env=[0,0,0];self.scan_start=self.m.read(8);self.index=0;self.wait_until=None

    def poll(self):
        if self.m.read(0x10)!=self.boot:raise RuntimeError('Controller boot changed during GAS10')
        if ((self.m.read(8)-self.scan_start)&0xffffffff)>30000:raise RuntimeError('GAS10 scan timed out')
        if self.index==10:
            if self.m.read(0x3b4)&3:return None
            self.scan_id=(self.scan_id+1)&0xffffffff
            words=[0]*44
            words[2:10]=[self.scan_id,PROFILE_ID,1,self.scan_start,self.scan_end,self.mask,1,DURATION_MS]
            words[10:20]=self.values;words[20:30]=self.times
            words[30]=sum(x<<(3*i) for i,x in enumerate(self.status))
            words[31:34]=self.env;words[34]=PROFILE_CRC
            for i,value in enumerate(words):self.m.write(0x300+4*i,value&0xffffffff)
            self.m.write(0x3b0,1)
            self.last_complete=dict(scan_id=self.scan_id,valid_mask=self.mask,gas10_ohm=list(self.values),point_timestamp_ms=list(self.times))
            done=self.last_complete;self._reset();return done
        self.m.write(0x3c4,self.index)
        if self.wait_until is None:
            if self.clock()<self.retry_after:return None
            try:
                delay=self.sensor.begin_profile(TEMPERATURES[self.index],DURATION_MS)
                self.wait_until=self.clock()+delay
            except (OSError,RuntimeError,ValueError) as exc:self._failed_point('begin',exc)
            return None
        if self.clock()<self.wait_until:return None
        try:
            values,flags=self.sensor.finish_profile()
            self.times[self.index]=self.m.read(8);self.status[self.index]=flags&7
            self.env=values[:3]
            self.values[self.index]=values[3] # Preserve measured-but-invalid raw points with a cleared mask.
            if flags==7 and values[3]>0:
                self.mask|=1<<self.index
            else:self.errors+=1
            # Keep old environment telemetry; never masquerade varying profiles as fixed320 gas.
            for i,value in enumerate(values[:3]+[0]):self.m.write(0x80+4*i,value&0xffffffff)
            self.m.write(0x98,flags&5)
            self._advance()
        except (OSError,RuntimeError,ValueError) as exc:self._failed_point('finish',exc)
        self.m.write(0x3c8,self.errors)
        return None

    def _advance(self):
        self.index+=1;self.wait_until=None
        if ((self.m.read(8)-self.scan_start)&0xffffffff)>30000:raise RuntimeError('GAS10 scan timed out')
        if self.index==10:self.scan_end=self.m.read(8)

    def _failed_point(self,phase,exc):
        self.last_error=dict(phase=phase,index=self.index,board_ms=self.m.read(8),error=type(exc).__name__+': '+str(exc))
        if self.clock()>=self._next_error_log:
            logging.getLogger(__name__).warning('GAS10 point failed: %s; total_errors=%s',self.last_error,self.errors+1)
            self._next_error_log=self.clock()+5
        # Persistent errors must not spin/publish hundreds of empty scans per
        # minute. Failed points remain zero/invalid, with no old-value filling.
        self.retry_after=self.clock()+(.1 if phase=='begin' else 0)
        self.errors+=1;self.times[self.index]=self.m.read(8)
        self.m.write(0x98,0);self.m.write(0x3c8,self.errors);self._advance()
