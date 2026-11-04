"""A simulated 50Hz PL enforces the real 500ms lease under network faults."""
import heapq
from pathlib import Path
import socket
import struct
import unittest
from unittest.mock import patch

from car.config import load_config
from car.host import Client
from car.protocol import crc16,signed32


class Clock:
    def __init__(self):self.now=1.0
    def monotonic(self):return self.now


class PLNetwork:
    def __init__(self,clock,delay=0,drop_first=False,drop_all=False,silence=False,boot_change=False,turn_complete=False,delay_from_start=False):
        self.clock=clock;self.delay=delay;self.drop_first=drop_first;self.drop_all=drop_all
        self.silence=silence;self.boot_change=boot_change;self.turn_complete=turn_complete
        self.delay_from_start=delay_from_start
        self.timeout=.2;self.next_frame=clock.now+.02;self.queue=[];self.seq=0
        self.boot=123;self.last_seq=0;self.action=0;self.mode=0;self.reason=0
        self.checked=0;self.status=0;self.keepalive=clock.now;self.started=None
        self.heartbeat_seen=False;self.dropped=False;self.tx=[];self.expired=[];self.starts=0

    def settimeout(self,value):self.timeout=value
    def close(self):pass

    def sendto(self,p,_address):
        if p==b'FCAR?':return
        seq,boot,issued=struct.unpack_from('<III',p,4);mode=p[16]
        p0=struct.unpack_from('<i',p,20)[0];ttl=struct.unpack_from('<H',p,18)[0]
        self.tx.append((self.clock.now,mode,seq))
        if mode==4:
            self.heartbeat_seen=True
            if self.drop_all or (self.drop_first and not self.dropped):self.dropped=True;return
        self.checked=seq
        if boot!=self.boot:self.status=1;return
        if mode==0:
            self.mode=0;self.action=0;self.reason=0;self.status=0
            if signed32(seq-self.last_seq)>0:self.last_seq=seq
            return
        if signed32(seq-self.last_seq)<=0:
            self.status=0 if seq==self.last_seq else 2;return
        if (round(self.clock.now*1000)-issued)&0xffffffff>ttl:self.status=3;return
        if mode==4:
            if not self.mode or (p0&0xffffffff)!=self.action:self.status=5;return
        else:
            self.mode=mode;self.action=seq;self.started=self.clock.now;self.starts+=1
        self.last_seq=seq;self.keepalive=self.clock.now;self.status=0

    def _frame(self,when):
        if self.boot_change and self.started is not None and when-self.started>=.5:
            self.boot=124;self.mode=0;self.action=0
        if self.mode and when-self.keepalive>=.5-1e-9:
            self.expired.append(when);self.mode=0;self.action=0;self.reason=2
        if self.turn_complete and self.mode==3 and when-self.started>=.6:
            self.mode=0;self.action=0;self.reason=4
        self.seq+=1;w=[0]*45
        w[0]=self.boot;w[1]=round(when*1000);w[2]=self.last_seq;w[3]=self.action
        w[4]=self.mode;w[6]=self.reason;w[7]=83;w[38]=self.status;w[42]=self.checked
        body=b'\xa5\x5a\x01\x81'+struct.pack('<I',self.seq)+struct.pack('<45I',*w)+b'\0\0'
        if not (self.silence and self.heartbeat_seen):
            delayed=self.heartbeat_seen or (self.delay_from_start and self.started is not None)
            arrival=when+(self.delay if delayed else 0)
            heapq.heappush(self.queue,(arrival,self.seq,body+struct.pack('<H',crc16(body))))

    def recvfrom(self,_size):
        deadline=self.clock.now+self.timeout
        while True:
            if self.queue and self.queue[0][0]<=self.clock.now+1e-9:
                return heapq.heappop(self.queue)[2],('car',4242)
            if self.clock.now>=deadline-1e-9:raise socket.timeout()
            delivery=self.queue[0][0] if self.queue else float('inf')
            event=min(self.next_frame,delivery,deadline)
            self.clock.now=max(self.clock.now,event)
            if self.next_frame<=self.clock.now+1e-9:
                self._frame(self.next_frame);self.next_frame+=.02


class HostKeepaliveTests(unittest.TestCase):
    def run_case(self,**faults):
        clock=Clock();link=PLNetwork(clock,**faults)
        config=load_config(Path(__file__).resolve().parents[1]/'config/car.json')
        return clock,link,config

    def test_delayed_telemetry_does_not_serialize_heartbeats_until_lease_expires(self):
        clock,link,c=self.run_case(delay=.4)
        with patch('car.host.time.monotonic',clock.monotonic):
            client=Client('car',c,sock=link);client.receive();client.hold(2,100,100,2)
        self.assertEqual(link.expired,[])
        self.assertEqual(link.mode,0)
        self.assertEqual(link.starts,1)
        heartbeats=[t for t,m,_ in link.tx if m==4]
        self.assertGreater(len(heartbeats),10)
        self.assertLess(max(b-a for a,b in zip(heartbeats,heartbeats[1:])),.2)

    def test_one_lost_heartbeat_recovers_but_total_loss_stops(self):
        clock,link,c=self.run_case(drop_first=True)
        with patch('car.host.time.monotonic',clock.monotonic):
            client=Client('car',c,sock=link);client.receive();client.hold(2,100,100,2)
        self.assertFalse(link.expired)
        self.assertTrue(link.dropped)
        clock,link,c=self.run_case(drop_all=True)
        with patch('car.host.time.monotonic',clock.monotonic):
            client=Client('car',c,sock=link);client.receive()
            with self.assertRaises((RuntimeError,TimeoutError)):client.hold(2,100,100,2)
        self.assertEqual(link.mode,0)
        self.assertEqual(link.tx[-1][1],0)

    def test_missing_telemetry_stops_and_stale_stop_does_not_wait_for_fresh_data(self):
        clock,link,c=self.run_case(silence=True)
        with patch('car.host.time.monotonic',clock.monotonic):
            client=Client('car',c,sock=link);client.receive()
            with self.assertRaises((RuntimeError,TimeoutError)):client.hold(2,100,100,2)
        stop_times=[t for t,m,_ in link.tx if m==0]
        self.assertTrue(stop_times)
        self.assertLess(stop_times[0]-link.started,.6)
        self.assertEqual(link.mode,0)

    def test_restart_does_not_replay_action_and_turn_completion_still_returns(self):
        clock,link,c=self.run_case(boot_change=True)
        with patch('car.host.time.monotonic',clock.monotonic):
            client=Client('car',c,sock=link);client.receive()
            with self.assertRaises(RuntimeError):client.hold(2,100,100,2)
        self.assertEqual(link.starts,1)
        clock,link,c=self.run_case(turn_complete=True)
        with patch('car.host.time.monotonic',clock.monotonic):
            client=Client('car',c,sock=link);client.receive();result=client.hold(3,1571,0,2)
        self.assertEqual(result['stop_reason'],'turn_complete')
        self.assertEqual(link.mode,0)

    def test_initial_ack_delay_is_included_in_requested_motion_duration(self):
        clock,link,c=self.run_case(delay=.25,delay_from_start=True)
        with patch('car.host.time.monotonic',clock.monotonic):
            client=Client('car',c,sock=link);client.receive();started=clock.now
            client.hold(2,100,100,2)
        stop_time=next(t for t,m,_ in link.tx if m==0)
        self.assertLessEqual(stop_time-started,2.01)
        self.assertFalse(link.expired)

    def test_cumulative_ack_across_sequence_wrap(self):
        clock,link,c=self.run_case(drop_first=True)
        link.last_seq=0xfffffffe
        with patch('car.host.time.monotonic',clock.monotonic):
            client=Client('car',c,sock=link);client.receive();client.hold(2,100,100,2)
        self.assertFalse(link.expired)
        self.assertEqual(link.starts,1)
        self.assertTrue(any(m==4 and seq<10 for _,m,seq in link.tx))
