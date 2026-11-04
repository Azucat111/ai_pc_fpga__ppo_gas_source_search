import struct
import unittest
import sys
from pathlib import Path
from car.protocol import crc16
from car.source_event import event_packet,decode_ack,SourceIndicator
from car.host import Client
from car.config import load_config
from test_host_keepalive import Clock,PLNetwork
from unittest.mock import patch

def ack(seq=1,boot=123,state=1,status=0):
    w=[boot,100,seq,status,1,state,seq,250,0x4c454431]+[0]*36
    body=b'\xa5\x5a\x01\x83'+struct.pack('<I45IH',1,*w,0)
    return body+struct.pack('<H',crc16(body))

class EventTests(unittest.TestCase):
    def test_operator_transport_blocks_all_motor_modes_including_stop(self):
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
        from windows_source_led_test import LEDOnlySocket
        from car.protocol import command
        class Sink:
            def __init__(self):self.packets=[]
            def sendto(self,p,a):self.packets.append(p);return len(p)
        sink=Sink();guard=LEDOnlySocket(sink)
        for mode in range(5):
            with self.assertRaises(RuntimeError):guard.sendto(command(1,123,100,mode),('car',4242))
        self.assertEqual(sink.packets,[])
        guard.sendto(b'FCAR?',('car',4242));guard.sendto(event_packet(1,123,100,1),('car',4242))
        self.assertEqual(len(sink.packets),2)
    def test_passive_audit_keeps_event_acks_out_of_state_and_gas_statistics(self):
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
        from ubuntu_link_audit import Audit
        audit=Audit(load_config('config/car.json'))
        record=audit.observe(ack(),1000)
        self.assertEqual(record['kind'],'source_ack')
        self.assertEqual(audit.unique_frames,0);self.assertEqual(audit.gas10_frames,0)
        self.assertEqual(audit.invalid_datagrams,0)
        self.assertEqual(audit.summary()['source_event_acks'],1)
    def test_independent_type_and_crc(self):
        p=event_packet(7,8,9,1)
        self.assertEqual(len(p),32);self.assertEqual(p[3],2)
        self.assertEqual(crc16(p[:30]),int.from_bytes(p[30:],'little'))
        self.assertEqual(struct.unpack_from('<II',p,20),(0,0))

    def test_ack_validation(self):
        self.assertTrue(decode_ack(ack())['confirmed'])
        bad=bytearray(ack());bad[90]^=1
        with self.assertRaises(ValueError):decode_ack(bad)

    def test_aux_ack_does_not_refresh_motion_state(self):
        clock=Clock();net=PLNetwork(clock);client=Client('car',load_config('config/car.json'),sock=net)
        with patch('time.monotonic',clock.monotonic):
            client.receive();received=client.received_at
            original=net.recvfrom;packets=[ack()]
            def receive(n):
                if packets:return packets.pop(),('car',4242)
                clock.now+=.02
                raise TimeoutError()
            net.recvfrom=receive
            with self.assertRaises(TimeoutError):client.receive(.01)
            self.assertEqual(client.received_at,received)
            self.assertEqual(client.latest_source_ack['request_seq'],1)

    def test_retry_preserves_packet_and_restart_cancels(self):
        clock=Clock();net=PLNetwork(clock);client=Client('car',load_config('config/car.json'),sock=net)
        with patch('time.monotonic',clock.monotonic):
            client.receive();client.latest['source_led_supported']=True
            sent=[];net.sendto=lambda p,a:sent.append(p)
            e=SourceIndicator(client);e.request(1);clock.now+=.11;e.poll()
            self.assertEqual(sent[0],sent[1]);self.assertTrue(all(p[3]==2 for p in sent))
            client.latest['boot_id']+=1
            with self.assertRaises(RuntimeError):e.poll()
            self.assertIsNone(e.pending)

    def test_no_unverified_old_firmware_commands(self):
        clock=Clock();net=PLNetwork(clock);client=Client('car',load_config('config/car.json'),sock=net)
        with patch('time.monotonic',clock.monotonic):
            client.receive()
            with self.assertRaises(RuntimeError):SourceIndicator(client).request(1)
        self.assertEqual(net.tx,[])

    def test_query_synchronizes_independent_sequence_and_rejection_is_visible(self):
        clock=Clock();net=PLNetwork(clock);client=Client('car',load_config('config/car.json'),sock=net)
        with patch('time.monotonic',clock.monotonic):
            client.receive();client.latest['source_led_supported']=True
            sent=[];net.sendto=lambda p,a:sent.append(p)
            e=SourceIndicator(client);e.request(2)
            client.latest_source_ack=decode_ack(ack(e.sequence))
            client.latest_source_ack['last_event_seq']=100
            self.assertIsNotNone(e.poll())
            e.request(1);self.assertEqual(e.sequence,101)
            client.latest_source_ack=decode_ack(ack(101,status=2))
            with self.assertRaisesRegex(RuntimeError,'rejected'):e.poll()
            self.assertTrue(all(p[3]==2 for p in sent))

    def test_timeout_does_not_create_another_event_or_stop(self):
        clock=Clock();net=PLNetwork(clock);client=Client('car',load_config('config/car.json'),sock=net)
        with patch('time.monotonic',clock.monotonic):
            client.receive();client.latest['source_led_supported']=True
            sent=[];net.sendto=lambda p,a:sent.append(p)
            e=SourceIndicator(client);e.request(1);clock.now+=.81
            with self.assertRaises(TimeoutError):e.poll()
            self.assertEqual(len(sent),1);self.assertIsNone(e.pending)

if __name__=='__main__':unittest.main()
