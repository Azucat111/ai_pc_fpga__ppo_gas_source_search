"""Passive capture preserves real wire data and never emits a car command."""
import json
from pathlib import Path
import socket
import struct
import sys
import tempfile
import threading
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from ubuntu_link_audit import Audit,collect
from car.config import load_config
from car.protocol import crc16


def frame(seq=1,stamp=1000,boot=123,flags=511,ages=(10,100,20,30)):
    w=[0]*45;w[0]=boot;w[1]=stamp;w[7]=flags
    w[8]=100;w[10]=200;w[16]=500;w[17]=20;w[18]=ages[0]
    w[19]=101;w[20]=202;w[21]=ages[2];w[22]=ages[3]
    w[23]=2500;w[24]=40000;w[25]=101300;w[26]=12345;w[27]=ages[1];w[28]=7
    body=b'\xa5\x5a\x01\x81'+struct.pack('<I',seq)+struct.pack('<45I',*w)+b'\0\0'
    return body+crc16(body).to_bytes(2,'little')


class UbuntuLinkAuditTests(unittest.TestCase):
    def setUp(self):self.config=load_config('config/car.json')

    def test_complete_v1_frame_preserved_without_inventing_missing_fields(self):
        a=Audit(self.config);packet=frame();r=a.observe(packet,1000000);t=r['telemetry']
        self.assertEqual(t['bme688']['gas_resistance_ohm'],12345)
        self.assertEqual([x['raw'] for x in t['tgs2602']],[101,202])
        self.assertEqual(r['state_payload_bytes'],180);self.assertEqual(r['frame_bytes'],192)
        self.assertEqual(r['board_timestamp_us_derived_from_ms'],1000000)
        self.assertEqual(r['board_time_resolution_us'],1000)
        self.assertEqual(r['reported_sample_commit_us_derived']['bme'],900000)
        self.assertNotIn('sensor_id',t['bme688']);self.assertNotIn('filtered',t['tgs2602'][0])
        self.assertFalse(a.summary()['requirements_complete'])

    def test_gaps_duplicates_and_old_frames_are_separate(self):
        a=Audit(self.config)
        for i,seq in enumerate((10,12,12,11)):a.observe(frame(seq,1000+i*20),i*20000000)
        s=a.summary();self.assertEqual(s['unique_frames'],2);self.assertEqual(s['duplicates_or_old'],2)
        self.assertEqual(s['boots'][0]['estimated_missing_telemetry_frames'],1)

    def test_board_timestamp_and_sequence_wrap_remain_continuous(self):
        a=Audit(self.config);a.observe(frame(0xffffffff,0xfffffff0),0)
        r=a.observe(frame(0,4),20000000)
        self.assertEqual(r['board_timestamp_us_derived_from_ms'],((1<<32)+4)*1000)
        self.assertEqual(a.summary()['boots'][0]['board_span_ms'],20)
        self.assertEqual(a.summary()['boots'][0]['estimated_missing_telemetry_frames'],0)

    def test_repeated_sensor_values_do_not_count_as_new_samples(self):
        a=Audit(self.config);a.observe(frame(1,1000,ages=(10,100,20,30)),0)
        a.observe(frame(2,1020,ages=(30,120,40,50)),20000000)
        self.assertEqual(a.summary()['observed_updates_by_reconstructed_commit_time'],dict(imu=1,bme=1,tgs0=1,tgs1=1))
        r=a.observe(frame(3,1040,flags=16|128|256),40000000)
        self.assertIsNone(r['telemetry']['imu_yaw'])
        self.assertIsNone(r['reported_sample_commit_us_derived']['imu'])
        self.assertIsNone(r['telemetry']['bme688']['gas_resistance_ohm'])
        self.assertEqual(a.summary()['bme_gas_usable_frames'],2)

    def test_corruption_and_router_replies_are_reported(self):
        a=Audit(self.config);packet=frame()
        a.observe(packet[:-1]+bytes([packet[-1]^1]),0);a.observe(b'bad',1)
        a.observe(b'FCAR_FULL',2)
        s=a.summary();self.assertEqual(s['crc_errors'],1);self.assertEqual(s['invalid_datagrams'],2)
        self.assertEqual(s['router_replies'],{'FCAR_FULL':1})

    def test_boot_change_does_not_create_fake_gap_or_cross_boot_time(self):
        a=Audit(self.config);a.observe(frame(100,10000,boot=1),0)
        a.observe(frame(1,50,boot=2),20000000)
        s=a.summary();self.assertEqual(len(s['boots']),2)
        self.assertTrue(all(x['estimated_missing_telemetry_frames']==0 for x in s['boots']))

    def test_actual_local_udp_capture_sends_only_registration_even_on_exit(self):
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as server,tempfile.TemporaryDirectory() as tmp:
            server.bind(('127.0.0.1',0));server.settimeout(.05)
            received=[];stop=threading.Event()
            def peer():
                while not stop.is_set():
                    try:data,address=server.recvfrom(4096)
                    except socket.timeout:continue
                    received.append(data)
                    server.sendto(frame(1,1000),address);server.sendto(frame(2,1020),address)
            thread=threading.Thread(target=peer);thread.start()
            try:r=collect('127.0.0.1',server.getsockname()[1],self.config,.15,Path(tmp)/'capture')
            finally:stop.set();thread.join(1)
            self.assertFalse(thread.is_alive());self.assertEqual(received,[b'FCAR?'])
            self.assertTrue(r['transport_check_passed']);self.assertEqual(r['motion_commands_sent'],0)
            self.assertEqual(r['stop_commands_sent'],0)
            raw=[json.loads(x) for x in (Path(tmp)/'capture/raw_datagrams.jsonl').read_text().splitlines()]
            decoded=[json.loads(x) for x in (Path(tmp)/'capture/decoded_frames.jsonl').read_text().splitlines()]
            self.assertEqual(bytes.fromhex(raw[0]['packet_hex']),frame())
            self.assertIn('pc_receive_timestamp_us',raw[0]);self.assertIn('pc_receive_monotonic_us',decoded[0])
            self.assertFalse(r['requirements_complete'])
