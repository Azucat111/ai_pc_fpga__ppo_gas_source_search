"""Exercise the real remote controller against a 50 Hz PL/500 ms lease model."""
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import patch
from car.config import load_config
from car.host import Client,ControlBusy
from car.protocol import crc16
from car.teleop import Teleop
from test_host_keepalive import Clock,PLNetwork
from test_multi_client import BusyNetwork

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import windows_remote

class CurrentNetwork(PLNetwork):
    def recvfrom(self,n):
        packet,source=super().recvfrom(n)
        body=bytearray(packet[:-2]);flags=struct.unpack_from('<I',body,36)[0]
        struct.pack_into('<I',body,36,flags|128)
        return bytes(body)+struct.pack('<H',crc16(body)),source

class TeleopTests(unittest.TestCase):
    def create(self,network_class=CurrentNetwork,**faults):
        clock=Clock();network=network_class(clock,**faults)
        client=Client('car',load_config('config/car.json'),sock=network)
        return clock,network,client,Teleop(client)

    def test_connect_is_observe_only_and_start_requires_fresh_state(self):
        clock,link,client,engine=self.create()
        with patch('time.monotonic',clock.monotonic):
            with self.assertRaises(RuntimeError):engine.start('w')
            engine.poll();self.assertEqual(link.tx,[])
            clock.now+=.2
            with self.assertRaises(RuntimeError):engine.start('w')
        self.assertEqual(link.starts,0)

    def test_hold_uses_one_action_heartbeats_and_release_stops(self):
        clock,link,client,engine=self.create()
        with patch('time.monotonic',clock.monotonic):
            engine.poll();engine.start('w');deadline=clock.now+2
            while clock.now<deadline:engine.poll()
            self.assertEqual(link.starts,1);self.assertFalse(link.expired)
            self.assertGreater(sum(m==4 for _,m,_ in link.tx),15)
            engine.stop()
            self.assertEqual(link.mode,0) # STOP datagram is immediate, no ACK wait
            while engine.owns:engine.poll()
        self.assertEqual(engine.phase,'idle')

    def test_stop_before_first_ack_never_restarts_motion(self):
        clock,link,client,engine=self.create()
        with patch('time.monotonic',clock.monotonic):
            engine.poll();engine.start('w');engine.stop()
            for _ in range(30):engine.poll()
        self.assertEqual(link.starts,1);self.assertEqual(link.mode,0)
        self.assertEqual([m for _,m,_ in link.tx],[2,0])

    def test_busy_abandons_without_interrupting_other_station(self):
        clock,link,client,engine=self.create(BusyNetwork)
        with patch('time.monotonic',clock.monotonic):
            engine.poll();client.latest['straight_gyro_heading']=True
            engine.start('w')
            with self.assertRaises(ControlBusy):engine.poll()
        self.assertFalse(engine.owns)
        self.assertEqual([m for _,m,_ in link.tx],[2])

    def test_lost_return_link_and_lost_heartbeat_are_detected(self):
        for faults in ({'silence':True},{'drop_all':True}):
            clock,link,client,engine=self.create(**faults)
            with patch('time.monotonic',clock.monotonic):
                engine.poll();engine.start('w');started=clock.now
                with self.assertRaises(TimeoutError):
                    while clock.now-started<2:engine.poll()
                engine.stop();engine.abandon()
            self.assertEqual(link.mode,0)
            self.assertLess(clock.now-started,.6)

    def test_boot_change_does_not_resume_or_stop_new_session(self):
        clock,link,client,engine=self.create(boot_change=True)
        with patch('time.monotonic',clock.monotonic):
            engine.poll();engine.start('w')
            with self.assertRaisesRegex(RuntimeError,'重启'):
                while clock.now<3:engine.poll()
        self.assertFalse(engine.owns);self.assertEqual(link.starts,1)
        self.assertFalse(any(m==0 for _,m,_ in link.tx))

    def test_other_controller_motion_is_not_interrupted_by_start_attempt(self):
        clock,link,client,engine=self.create()
        link.mode=2
        with patch('time.monotonic',clock.monotonic):
            engine.poll()
            with self.assertRaises(ControlBusy):engine.start('w')
            engine.stop() # normal release/close of an observer is not an emergency STOP
        self.assertEqual(link.tx,[])

    def test_gui_freeze_deadman_stops_once_without_restarting_held_key(self):
        clock,link,client,engine=self.create()
        intent=windows_remote.Intent();intent.alive=clock.now;intent.set('w')
        closing=windows_remote.threading.Event();events=windows_remote.queue.Queue()
        original=link.recvfrom
        def receive(n):
            result=original(n)
            if clock.now>2:closing.set()
            return result
        link.recvfrom=receive
        with patch('time.monotonic',clock.monotonic),patch.object(windows_remote,'Client',return_value=client):
            windows_remote.worker(intent,events,closing,'car')
        self.assertEqual(link.starts,1);self.assertEqual(link.mode,0)
        stop=next(t for t,m,_ in link.tx if m==0)
        self.assertLess(stop-1,.32)
