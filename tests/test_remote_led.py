import struct
import unittest
import socket
from unittest.mock import patch
from car.config import load_config,rpm_target
from car.host import Client
from car.protocol import crc16
from car.teleop import Teleop,STRAIGHT_RPM,TURN_RPM
from car.timed_source_led import TimedSourceLED
from test_host_keepalive import Clock,PLNetwork


class LEDNetwork(PLNetwork):
    def __init__(self, clock):
        super().__init__(clock);self.acks=[];self.led=False;self.event_seq=100
        self.events=[];self.lose_clear_ack=False;self.motion_packets=[]

    def sendto(self,p,address):
        if p==b'FCAR?':return
        if p[3]!=2:
            self.motion_packets.append(p);return super().sendto(p,address)
        seq,boot,_=struct.unpack_from('<III',p,4);event=p[16]
        self.events.append((self.clock.now,event,seq))
        status=0
        if event!=2:
            if seq<=self.event_seq:status=2
            else:self.event_seq=seq;self.led=bool(event)
        if event==0 and self.lose_clear_ack:return
        words=[boot,round(self.clock.now*1000),seq,status,event,int(self.led),self.event_seq,250,0x4c454431]+[0]*36
        body=b'\xa5\x5a\x01\x83'+struct.pack('<I45IH',len(self.events),*words,0)
        self.acks.append(body+struct.pack('<H',crc16(body)))

    def recvfrom(self,n):
        if self.acks:return self.acks.pop(0),('car',4242)
        p,a=super().recvfrom(n);body=bytearray(p[:-2])
        flags=struct.unpack_from('<I',body,36)[0]
        struct.pack_into('<I',body,36,flags|128|1024)
        return bytes(body)+struct.pack('<H',crc16(body)),a


class RemoteLEDTests(unittest.TestCase):
    def create(self):
        clock=Clock();link=LEDNetwork(clock)
        client=Client('car',load_config('config/car.json'),sock=link)
        led=TimedSourceLED(client,lambda s:None)
        return clock,link,client,led

    def spin(self,clock,client,led,seconds):
        end=clock.now+seconds
        while clock.now<end:client.receive(.02);led.poll()

    def test_five_seconds_and_independent_sequence(self):
        clock,link,client,led=self.create()
        with patch('time.monotonic',clock.monotonic):
            client.receive();led.start();self.spin(clock,client,led,6)
        events=[e for e in link.events if e[1]!=2]
        self.assertEqual([e[1] for e in events],[1,0])
        self.assertEqual(events[0][2],101)
        self.assertAlmostEqual(events[1][0]-events[0][0],5,delta=.03)
        self.assertFalse(led.active);self.assertFalse(link.led)
        self.assertEqual(link.motion_packets,[])

    def test_led_does_not_interrupt_movement_heartbeats(self):
        clock,link,client,led=self.create();engine=Teleop(client)
        with patch('time.monotonic',clock.monotonic):
            engine.poll();engine.start('w');led.start();end=clock.now+6
            while clock.now<end:engine.poll();led.poll()
            self.assertEqual(engine.phase,'driving');engine.stop()
            while engine.owns:engine.poll()
        self.assertFalse(link.expired);self.assertEqual(link.starts,1)
        self.assertFalse(link.led)
        p=next(p for p in link.motion_packets if p[16]==2)
        c=client.config
        self.assertEqual(struct.unpack_from('<ii',p,20),(rpm_target(40,c,'left'),rpm_target(40,c,'right')))

    def test_close_during_flash_clears(self):
        clock,link,client,led=self.create()
        with patch('time.monotonic',clock.monotonic):
            client.receive();led.start();self.spin(clock,client,led,.2)
            self.assertTrue(link.led);led.cancel();self.spin(clock,client,led,.2)
        self.assertFalse(link.led);self.assertFalse(led.active)
        self.assertEqual(link.motion_packets,[])

    def test_clear_ack_loss_retries_after_recovery(self):
        clock,link,client,led=self.create()
        with patch('time.monotonic',clock.monotonic):
            client.receive();led.start();link.lose_clear_ack=True
            self.spin(clock,client,led,6);self.assertTrue(led.active)
            link.lose_clear_ack=False;self.spin(clock,client,led,2)
        self.assertFalse(link.led);self.assertFalse(led.active)
        self.assertEqual(link.motion_packets,[])

    def test_old_firmware_refuses_led_without_motion(self):
        clock,link,client,led=self.create()
        with patch('time.monotonic',clock.monotonic):
            client.receive();client.latest['source_led_supported']=False;led.start()
        self.assertFalse(led.active);self.assertEqual(link.events,[])

    def test_speed_and_signed_directions(self):
        self.assertEqual((STRAIGHT_RPM,TURN_RPM),(40,20))
        for key,mode,left,right in [('w',2,40,40),('s',2,-40,-40),('a',1,-20,20),('d',1,20,-20)]:
            clock,link,client,_=self.create();engine=Teleop(client)
            with patch('time.monotonic',clock.monotonic):engine.poll();engine.start(key)
            packet=link.motion_packets[-1]
            self.assertEqual(packet[16],mode)
            self.assertEqual(struct.unpack_from('<ii',packet,20),(rpm_target(left,client.config,'left'),rpm_target(right,client.config,'right')))

    def test_event_ack_does_not_refresh_motion_telemetry(self):
        clock,link,client,led=self.create()
        with patch('time.monotonic',clock.monotonic):
            client.receive();before=client.received_at;led.start()
            def ack_only(n):
                if link.acks:return link.acks.pop(0),('car',4242)
                clock.now+=.02;raise socket.timeout()
            link.recvfrom=ack_only
            with self.assertRaises(TimeoutError):client.receive(.02)
        self.assertEqual(client.received_at,before)
        self.assertIsNotNone(client.latest_source_ack)

    def test_led_mouse_click_keeps_keyboard_direction(self):
        import windows_remote
        window=windows_remote.Window.__new__(windows_remote.Window)
        window.mouse_held=False;window.current='w';window.intent=windows_remote.Intent()
        window.intent.set('w');window.intent.request_led();window.release_mouse()
        self.assertEqual(window.current,'w');self.assertEqual(window.intent.snapshot()[0],'w')
        self.assertTrue(window.intent.take_led())


if __name__=='__main__':unittest.main()
