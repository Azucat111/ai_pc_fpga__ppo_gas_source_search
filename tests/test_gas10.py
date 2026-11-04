"""GAS10 acquisition is isolated from motion and never fabricates missing points."""
import json
from pathlib import Path
import struct
import socket
import tempfile
import threading
import sys
import unittest
from car.gas10 import decode_gas10,PROFILE_ID,PROFILE_CRC,TEMPERATURES
from car.gas10_board import Scanner
from car.config import load_config
from car.protocol import crc16
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from ubuntu_link_audit import Audit,collect


class Memory:
    def __init__(self):self.now=1.;self.reg={0xd8:0x47415331,0x10:123};self.writes=[]
    def read(self,a):return int(self.now*1000)&0xffffffff if a==8 else self.reg.get(a,0)
    def write(self,a,v):self.reg[a]=v;self.writes.append((a,v))
    def packet(self,change=None):
        w=[self.reg.get(0x300+i*4,0) for i in range(44)]+[0];w[0]=123;w[1]=self.read(8)
        if change:change(w)
        body=b'\xa5\x5a\x01\x82'+struct.pack('<I',1)+struct.pack('<45I',*w)+b'\0\0'
        return body+crc16(body).to_bytes(2,'little')


class Sensor:
    def __init__(self,bad=None):self.started=[];self.finished=0;self.bad=bad
    def begin_profile(self,temp,duration):self.started.append((temp,duration));return .15
    def finish_profile(self):
        i=self.finished;self.finished+=1
        if i==self.bad:raise OSError('injected read failure')
        return [2500,45000,101325,1000+i],7


class Gas10Tests(unittest.TestCase):
    def complete(self,bad=None):
        m=Memory();sensor=Sensor(bad);scan=Scanner(sensor,m,clock=lambda:m.now)
        for _ in range(10):
            scan.poll();m.now+=.151;scan.poll()
        result=scan.poll()
        return m,sensor,scan,result

    def test_valid_scan_and_point_order_are_not_a_classification(self):
        m,s,scan,result=self.complete();sample=decode_gas10(m.packet())
        self.assertEqual(s.started,list(zip(TEMPERATURES,[100]*10)))
        self.assertTrue(sample['acquisition_valid']);self.assertEqual(sample['gas_valid_mask'],1023)
        self.assertEqual(sample['gas10_ohm'],list(range(1000,1010)))
        self.assertFalse(sample['classification_valid']);self.assertIsNone(sample['alcohol_prob'])
        self.assertFalse(sample['model_profile_equivalence_verified'])
        self.assertTrue(all(a not in (0x14,0x10) for a,v in m.writes))
        self.assertEqual(m.reg[0x98]&2,0) # Legacy consumers cannot treat changing profiles as fixed320.

    def test_heating_wait_does_not_sleep_or_repeat_mailbox_work(self):
        m=Memory();s=Sensor();scan=Scanner(s,m,clock=lambda:m.now);scan.poll()
        for _ in range(100):scan.poll()
        self.assertEqual(len(s.started),1);self.assertEqual(s.finished,0)
        m.now+=.151;scan.poll();self.assertEqual(s.finished,1)

    def test_missing_point_stays_invalid_and_next_scan_starts_empty(self):
        m,s,scan,result=self.complete(bad=3);sample=decode_gas10(m.packet())
        self.assertFalse(sample['acquisition_valid']);self.assertEqual(sample['gas_valid_mask'],1023^(1<<3))
        self.assertEqual(sample['gas10_ohm'][3],0);self.assertEqual(scan.values,[0]*10)
        self.assertEqual(scan.mask,0);self.assertEqual(scan.errors,1)

    def test_mailbox_backpressure_does_not_overwrite_or_begin_next_scan(self):
        m=Memory();s=Sensor();scan=Scanner(s,m,clock=lambda:m.now)
        for _ in range(10):scan.poll();m.now+=.151;scan.poll()
        m.reg[0x3b4]=1
        for _ in range(4):self.assertIsNone(scan.poll())
        self.assertEqual(len(s.started),10);self.assertNotIn(0x3b0,m.reg)
        m.reg[0x3b4]=0;self.assertIsNotNone(scan.poll())

    def test_controller_restart_rejects_partial_scan(self):
        m=Memory();scan=Scanner(Sensor(),m,clock=lambda:m.now);scan.poll();m.reg[0x10]=456
        with self.assertRaisesRegex(RuntimeError,'boot'):scan.poll()
        self.assertNotIn(0x3b0,m.reg)

    def test_unknown_profile_is_recorded_but_not_model_ready(self):
        m,_,_,_=self.complete();sample=decode_gas10(m.packet(lambda w:w.__setitem__(34,0)))
        self.assertFalse(sample['profile_known']);self.assertFalse(sample['acquisition_valid'])

    def test_corrupt_mask_timestamp_and_crc_are_rejected(self):
        m,_,_,_=self.complete()
        for change in (lambda w:w.__setitem__(7,2047),lambda w:w.__setitem__(10,0),lambda w:w.__setitem__(20,w[6]+1)):
            with self.assertRaises(ValueError):decode_gas10(m.packet(change))
        packet=m.packet()
        with self.assertRaises(ValueError):decode_gas10(packet[:-1]+bytes([packet[-1]^1]))

    def test_audit_accepts_auxiliary_frames_without_polluting_state_sequences(self):
        m,_,_,_=self.complete();a=Audit(load_config('config/car.json'))
        for _ in range(2):self.assertEqual(a.observe(m.packet(),1)['kind'],'gas10')
        r=a.summary();self.assertEqual(r['gas10_frames'],2);self.assertEqual(r['gas10_valid_unique_scans'],1)
        self.assertEqual(r['unique_frames'],0);self.assertEqual(r['invalid_datagrams'],0)

    def test_unstable_measured_point_is_preserved_but_cannot_become_valid(self):
        class Unstable(Sensor):
            def finish_profile(self):
                values,flags=super().finish_profile()
                return values,3 if self.finished==1 else flags
        m=Memory();sensor=Unstable();scanner=Scanner(sensor,m,clock=lambda:m.now)
        for _ in range(10):scanner.poll();m.now+=.151;scanner.poll()
        scanner.poll();sample=decode_gas10(m.packet())
        self.assertEqual(sample['gas10_ohm'][0],1000)
        self.assertEqual(sample['gas_valid_mask'],1022)
        self.assertFalse(sample['acquisition_valid'])

    def test_aux_frames_do_not_replace_control_state_and_cache_clears_on_boot(self):
        from car.host import Client
        from test_software import packet as state_packet
        m,_,_,_=self.complete();gas=m.packet()
        def state(seq,boot):
            w=[0]*45;w[0]=boot;w[1]=1000;w[7]=83
            packet=bytearray(state_packet(w));struct.pack_into('<I',packet,4,seq)
            packet[-2:]=crc16(packet[:-2]).to_bytes(2,'little');return bytes(packet)
        class Socket:
            def __init__(self):self.data=[state(1,123),gas,state(2,123),state(1,456),gas,state(2,456)]
            def settimeout(self,_):pass
            def sendto(self,*_):pass
            def recvfrom(self,_):return self.data.pop(0),('car',4242)
            def close(self):pass
        client=Client('car',load_config('config/car.json'),sock=Socket())
        client.receive();client.receive()
        self.assertIsNotNone(client.latest_gas10);self.assertEqual(client.latest['seq'],2)
        client.receive();self.assertIsNone(client.latest_gas10)
        client.receive();self.assertIsNone(client.latest_gas10)

    def test_stalled_acquisition_cannot_publish_an_expired_scan(self):
        m=Memory();scan=Scanner(Sensor(),m,clock=lambda:m.now);scan.poll();m.now+=31
        with self.assertRaisesRegex(RuntimeError,'timed out'):scan.poll()
        self.assertNotIn(0x3b0,m.reg)

    def test_profile_start_failures_are_logged_and_backed_off_without_fake_data(self):
        class Broken(Sensor):
            def begin_profile(self,temp,duration):
                self.started.append((temp,duration))
                raise RuntimeError('PL I2C mailbox is already busy')
        m=Memory();sensor=Broken();scanner=Scanner(sensor,m,clock=lambda:m.now)
        with self.assertLogs('car.gas10_board',level='WARNING') as logs:
            scanner.poll()
            for _ in range(10):
                m.now+=.005;scanner.poll()
            self.assertEqual(len(sensor.started),1)
            for _ in range(9):
                m.now+=.101;scanner.poll()
            scanner.poll()
        self.assertEqual(len(logs.output),1)
        self.assertIn('mailbox is already busy',logs.output[0])
        self.assertEqual(scanner.last_error['phase'],'begin')
        self.assertEqual(scanner.errors,10)
        sample=decode_gas10(m.packet())
        self.assertFalse(sample['acquisition_valid'])
        self.assertEqual(sample['gas10_ohm'],[0]*10)
        self.assertGreaterEqual(sample['scan_duration_ms'],900)

    def test_profile_read_failure_records_which_phase_and_point_failed(self):
        with self.assertLogs('car.gas10_board',level='WARNING') as logs:
            m,s,scanner,result=self.complete(bad=3)
        self.assertEqual(scanner.last_error['phase'],'finish')
        self.assertEqual(scanner.last_error['index'],3)
        self.assertIn('injected read failure',logs.output[0])
        self.assertEqual(result['valid_mask'],1023^(1<<3))

    def test_udp_audit_separates_valid_frames_from_failed_gas_acquisition(self):
        from test_ubuntu_link_audit import frame
        m,_,_,_=self.complete()
        for good in (False,True):
            with self.subTest(good=good),socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as server,tempfile.TemporaryDirectory() as tmp:
                server.bind(('127.0.0.1',0));server.settimeout(.02)
                stop=threading.Event();received=[]
                def peer():
                    while not stop.is_set():
                        try:packet,address=server.recvfrom(4096)
                        except socket.timeout:continue
                        received.append(packet)
                        server.sendto(frame(1,1000),address);server.sendto(frame(2,1020),address)
                        for scan in range(1,4):
                            def change(w):
                                w[2]=scan
                                if not good:w[7]=0;w[10:20]=[0]*10;w[30]=0
                            server.sendto(m.packet(change),address)
                worker=threading.Thread(target=peer);worker.start()
                try:result=collect('127.0.0.1',server.getsockname()[1],load_config('config/car.json'),.15,Path(tmp)/'capture',require_gas10=True)
                finally:stop.set();worker.join(1)
                self.assertFalse(worker.is_alive());self.assertEqual(received,[b'FCAR?'])
                self.assertTrue(result['telemetry_transport_check_passed'])
                self.assertTrue(result['gas10_frame_transport_check_passed'])
                self.assertEqual(result['gas10_acquisition_check_passed'],good)
                self.assertEqual(result['capture_check_passed'],good)
                self.assertEqual(result['gas10_valid_unique_scans'],3 if good else 0)
