import json
import math
import struct
import unittest
import sys
from types import SimpleNamespace
from unittest.mock import patch
import ctypes as C
from pathlib import Path
from car.config import load_config,registers,rpm_target,velocity_targets
from car.protocol import command,crc16,decode,decode_snapshot,signed32,signed64
from car.board import PLBus,configure

ROOT=Path(__file__).resolve().parents[1]

def packet(words):
    body=b'\xa5\x5a\x01\x81'+struct.pack('<I',7)+struct.pack('<45I',*[x&0xffffffff for x in words])+b'\0\0'
    return body+struct.pack('<H',crc16(body))

class SoftwareTests(unittest.TestCase):
    def setUp(self):self.c=load_config(ROOT/'config/car.json')
    def test_motor_debug_decoder(self):
        from car.debug import ADDRESSES,decode_registers
        words={address:0 for address in ADDRESSES}
        words.update({0x64:0x44424731,0x68:5000000,0x70:0xffff0000,0x74:144179,
                      0x78:(219<<16)|0xff9c,0xac:(219<<16)|0xff9c,
                      0x7c:17,0xb0:600000,0xa0:1,0xa4:0xffff0000,0xa8:144179})
        result=decode_registers(words)
        self.assertEqual(result['duty_permille'],{'left':-100,'right':219})
        self.assertEqual(result['last_nonzero_duty_permille'],result['duty_permille'])
        self.assertEqual(result['target_counts_per_10ms']['left'],-1)
        self.assertAlmostEqual(result['target_counts_per_10ms']['right'],2.2,places=5)
        self.assertTrue(result['drive_active'] and result['output_logic']['AIN1'])
        self.assertFalse(result['output_logic']['AIN2'])
        self.assertEqual(result['output_rising_edges']['AIN1'],600000)
        self.assertEqual(result['last_received_motion']['p0'],-65536)
        self.assertFalse(result['imu_last_good_supported'])
        words[0xcc]=0x494d5531
        self.assertTrue(decode_registers(words)['imu_last_good_supported'])
        self.assertFalse(decode_registers(words)['imu_plausibility_supported'])
        words[0xcc]=0x494d5532
        self.assertTrue(decode_registers(words)['imu_last_good_supported'])
        self.assertTrue(decode_registers(words)['imu_plausibility_supported'])
        self.assertFalse(decode_registers(words)['straight_gyro_supported'])
        words[0xd0]=0x47595231
        self.assertTrue(decode_registers(words)['straight_gyro_supported'])
        self.assertFalse(decode_registers(words)['turn_gyro_supported'])
        words[0xd4]=0x47595232
        self.assertTrue(decode_registers(words)['turn_gyro_supported'])
        words[0x64]=0
        with self.assertRaises(RuntimeError):decode_registers(words)
    def test_turn_gyro_capability_does_not_change_wire_layout(self):
        w=[0]*45;w[7]=128|256
        decoded=decode(packet(w),self.c)
        self.assertTrue(decoded['straight_gyro_heading'])
        self.assertTrue(decoded['turn_gyro_heading'])
        w[7]=128
        self.assertFalse(decode(packet(w),self.c)['turn_gyro_heading'])
    def test_finite_board_service_clears_motor_permission(self):
        from car.board import main
        class Memory:
            def __init__(self):self.values={0:0x43415231,4:0x00010000,0xd0:0x47595231,0xd4:0x47595232};self.permission_writes=[]
            def read(self,address):return self.values.get(address,0)
            def write(self,address,value):
                self.values[address]=value
                if address==0x14:self.permission_writes.append(value)
        memory=Memory();clock=[0.0]
        def sleep(seconds):
            clock[0]+=seconds
            if clock[0]>3:raise AssertionError('Finite service failed to honor its deadline')
        fake=SimpleNamespace(Overlay=lambda *_:SimpleNamespace(ip_dict={'car_control':
            {'phys_addr':0x43c00000,'addr_range':0x1000}}),MMIO=lambda *_:memory)
        with patch.dict(sys.modules,{'pynq':fake}), \
             patch.object(sys,'argv',['car-board','--enable','--no-bme','--seconds','0.1']), \
             patch('car.board.load_config',return_value=dict(self.c,wiring_confirmed=True,imu_mount_confirmed=False)), \
             patch('car.board.PLBus') as bus, \
             patch('car.board.time.monotonic',side_effect=lambda:clock[0]), \
             patch('car.board.time.sleep',side_effect=sleep),patch('builtins.print'):
            bus.return_value.transfer.return_value=b'\x01\0\0'
            main()
        self.assertIn(1,memory.permission_writes)
        self.assertEqual(memory.permission_writes[-1],0)
        self.assertEqual(memory.values[0x98],0)
    def test_crc_known_vector(self):self.assertEqual(crc16(b'123456789'),0x29b1)
    def test_command_layout(self):
        p=command(5,0x12345678,123,1,-131072,65536)
        self.assertEqual(len(p),32)
        self.assertEqual(struct.unpack_from('<ii',p,20),(-131072,65536))
        self.assertEqual(struct.unpack_from('<I',p,8)[0],0x12345678)
        self.assertEqual(crc16(p[:30]),struct.unpack_from('<H',p,30)[0])
    def test_unknown_geometry(self):
        self.c.update(wheel_diameter_left_m=None,wheel_diameter_right_m=None,track_width_m=None)
        self.assertEqual(registers(self.c)[0x60],0)
        with self.assertRaises(ValueError):velocity_targets(.1,0,self.c)
        w=[0]*45;w[8]=0xffffffff;w[9]=0xffffffff
        d=decode(packet(w),self.c)
        self.assertEqual(d['encoder_left_count'],-1)
        for key in ('linear_velocity','angular_velocity','wheel_speed_left','distance_m'):
            self.assertIsNone(d[key])
        self.assertIsNone(d['imu_yaw']);self.assertIsNone(d['tgs2602'][0]['raw'])
    def test_geometry_and_counts(self):
        c=self.c;c.update(wheel_diameter_left_m=.065,wheel_diameter_right_m=.065,track_width_m=.15,max_wheel_rpm=180,imu_mount_confirmed=False)
        r=registers(c);self.assertEqual(r[0x60],1)
        self.assertAlmostEqual(r[0x40]/65536,math.pi*.065/1320*1e6,places=4)
        self.assertEqual(rpm_target(60,c,'left'),round(13.2*65536))
        left,right=velocity_targets(0,.5,c);self.assertLess(left,0);self.assertGreater(right,0)
        with self.assertRaises(ValueError):velocity_targets(100,0,c)
    def test_assembled_car_parameters(self):
        c=self.c;r=registers(c)
        self.assertEqual((c['wheel_diameter_left_m'],c['wheel_diameter_right_m'],c['track_width_m']),(.067,.067,.170))
        self.assertFalse(c['m1_is_left'])
        self.assertTrue(c['encoder_m1_ab_swapped'] and c['encoder_m2_ab_swapped'])
        self.assertTrue(c['wiring_confirmed'])  # Both motor directions and running encoder feedback verified.
        self.assertEqual(c['duty_limit_permille'],700)
        self.assertEqual(c['max_wheel_rpm'],40)
        self.assertFalse(c['encoder_m1_invert'])
        self.assertTrue(c['encoder_m2_invert'])  # Forward hand-turn on the mirrored left motor.
        self.assertFalse(c['motor_m1_invert'])
        self.assertTrue(c['motor_m2_invert'])  # Left motor positive command was observed backward.
        self.assertEqual(r[0x18],0b10011)  # M1 right; encoder correction and M2 motor inversion.
        self.assertTrue(c['imu_mount_confirmed'])  # Level installation; manual left turn yaw/gyro signs verified.
        self.assertEqual(r[0x60],3)
        for addr in (0x40,0x44):
            # One full revolution must cover the measured circumference.
            self.assertAlmostEqual(r[addr]/65536*1320/1e6,math.pi*.067,places=7)
        left,right=velocity_targets(.05,0,c)
        self.assertEqual(left,right)
        self.assertAlmostEqual(left/65536/.01/1320*math.pi*.067,.05,places=6)
        left,right=velocity_targets(0,.5,c)
        vl=left/65536/.01/1320*math.pi*.067
        vr=right/65536/.01/1320*math.pi*.067
        self.assertAlmostEqual((vr-vl)/.170,.5,places=5)
    def test_encoder_wiring_correction_and_forward_trim(self):
        from itertools import product
        for swap1,swap2,trim1,trim2,left in product((False,True),repeat=5):
            with self.subTest(swap1=swap1,swap2=swap2,trim1=trim1,trim2=trim2,left=left):
                c=dict(self.c,m1_is_left=left,encoder_m1_ab_swapped=swap1,encoder_m2_ab_swapped=swap2,
                       encoder_m1_invert=trim1,encoder_m2_invert=trim2,
                       motor_m1_invert=True,motor_m2_invert=False)
                flags=registers(c)[0x18]
                self.assertEqual(bool(flags&1),not left)
                self.assertEqual(bool(flags&2),swap1!=trim1)
                self.assertEqual(bool(flags&4),swap2!=trim2)
                self.assertEqual(flags&0b11000,0b01000)
    def test_legacy_encoder_config(self):
        c=dict(self.c,encoder_m1_invert=True,encoder_m2_invert=False,
               motor_m1_invert=False,motor_m2_invert=False)
        del c['encoder_m1_ab_swapped'];del c['encoder_m2_ab_swapped']
        with patch('car.config.Path.read_text',return_value=json.dumps(c)):
            loaded=load_config('legacy.json')
        self.assertFalse(loaded['encoder_m1_ab_swapped'])
        self.assertFalse(loaded['encoder_m2_ab_swapped'])
        self.assertEqual(registers(loaded)[0x18],0b00011)
        self.assertEqual(registers(c),registers(loaded))
    def test_encoder_wiring_flags_require_booleans(self):
        for key in ('encoder_m1_ab_swapped','encoder_m2_ab_swapped'):
            for value in ('true',1,None):
                with self.subTest(key=key,value=value):
                    with patch('car.config.Path.read_text',return_value=json.dumps(dict(self.c,**{key:value}))):
                        with self.assertRaises(ValueError):load_config('invalid.json')
    def test_telemetry_units(self):
        c=self.c;c.update(wheel_diameter_left_m=.065,wheel_diameter_right_m=.065,track_width_m=.15)
        w=[0]*45;w[7]=127;w[14]=w[15]=65536;w[16]=-1571;w[17]=500
        w[19]=16384;w[20]=-2;w[23]=2534;w[24]=45678;w[25]=100123;w[26]=123456;w[28]=7
        w[33]=100000;w[34]=-500;w[37]=20
        d=decode(packet(w),c)
        self.assertEqual(d['imu_yaw'],-1.571);self.assertEqual(d['imu_gyro_z'],.5)
        self.assertEqual(d['tgs2602'][0]['voltage_v'],3.072)
        self.assertEqual(d['bme688']['humidity_percent'],45.678)
        self.assertEqual(d['linear_velocity'],.1);self.assertEqual(d['angular_velocity'],-.5)
        self.assertEqual(d['bme688']['gas_resistance_ohm'],123456)
        json.dumps(d,allow_nan=False)
        w[28]=1;self.assertIsNone(decode(packet(w),c)['bme688']['gas_resistance_ohm'])
    def test_bad_packet(self):
        p=bytearray(packet([0]*45));p[55]^=1
        with self.assertRaises(ValueError):decode(p,self.c)
        with self.assertRaises(ValueError):decode(p[:-1],self.c)
    def test_local_axi_snapshot(self):
        w=[0]*45;w[7]=16;w[8]=0xffffffff;w[9]=0xffffffff;w[10]=1320;w[14]=65536
        local=decode_snapshot(7,w,self.c)
        self.assertEqual(local,decode(packet(w),self.c))
        self.assertEqual(local['encoder_left_count'],-1)
        self.assertEqual(local['encoder_right_count'],1320)
        self.assertFalse(local['enabled'])
        with self.assertRaises(ValueError):decode_snapshot(7,w[:-1],self.c)
    def test_hardware_telemetry_vector(self):
        path=ROOT/'build/tests/telemetry.hex'
        if not path.exists():self.skipTest('RTL serializer test has not run')
        p=bytes(int(x,16) for x in path.read_text().split())
        self.assertEqual(p[:4],b'\xa5\x5a\x01\x81')
        self.assertEqual(struct.unpack_from('<45I',p,8),tuple(range(45)))
        self.assertEqual(struct.unpack_from('<I',p,4)[0],0x12345678)
        self.assertEqual(crc16(p[:-2]),int.from_bytes(p[-2:],'little'))
    def test_full_core_uart_telemetry(self):
        path=ROOT/'build/tests/integration_telemetry.hex'
        if not path.exists():self.skipTest('RTL integration test has not run')
        d=decode(bytes(int(x,16) for x in path.read_text().split()),self.c)
        self.assertEqual(d['boot_id'],0x12345678)
        self.assertEqual(d['encoder_left_count'],4)
        self.assertEqual(d['encoder_right_count'],0)
        self.assertEqual(d['last_action'],'wheel_velocity')
        self.assertEqual(d['last_command_seq'],1)
        self.assertTrue(d['enabled'])
        self.assertIsNone(d['linear_velocity'])
    def test_no_motion_without_configuration(self):
        class IO:
            writes=[]
            def read(self,a):return {0:0x43415231,4:0x10000}.get(a,0)
            def write(self,a,v):self.writes.append((a,v))
        io=IO()
        with self.assertRaises(ValueError):configure(io,dict(self.c,wiring_confirmed=False),enable=True)
        self.assertEqual(io.writes,[(0x14,0)])
    def test_mailbox(self):
        class IO:
            def __init__(self):self.data={};self.started=False
            def write(self,a,v):self.data[a]=v;self.started=self.started or a==0x100
            def read(self,a):
                if a==0x110:return 2 if self.started else 0
                return 0x44332211 if a==0x140 else 0
        io=IO();bus=PLBus(io)
        self.assertEqual(bus.transfer(0x23,0x26,b'\x55\x66',3),b'\x11\x22\x33')
        self.assertEqual(io.data[0x120],0x6655);self.assertEqual(io.data[0x10c],0x302)
        with self.assertRaises(ValueError):bus.transfer(0x23,0,read=33)
    def test_signed_arithmetic(self):
        self.assertEqual(signed32(0xffffffff),-1)
        self.assertEqual(signed32(0-0xffffffff),1)
        self.assertEqual(signed32(-1),-1)
        self.assertEqual(signed64(0,0x80000000),-(1<<63))
    def test_independent_imu_signs(self):
        self.c['imu_yaw_invert']=True
        self.assertEqual(registers(self.c)[0x54],1)
        self.c['imu_yaw_invert']=False;self.c['imu_gyro_z_invert']=True
        self.assertEqual(registers(self.c)[0x54],2)
    def test_bosch_callback_bridge(self):
        from car.bme import BME688
        class Function:
            def __init__(self,fn):self.fn=fn
            def __call__(self,*args):return self.fn(*args)
        class Library:
            def __init__(self):
                self.car_bme_init=Function(self.init)
                self.car_bme_sample=Function(self.sample)
            def init(self,*callbacks):self.callbacks=callbacks;return 0
            def sample(self,out):
                s=out._obj;s.temperature=25.5;s.humidity=50;s.pressure=101325;s.gas=100000;s.status=0xb0;return 0
        class Bus:
            def __init__(self):self.calls=[]
            def transfer(self,address,reg,write=b'',read=0):
                self.calls.append((address,reg,write,read))
                return bytes((reg+i)&255 for i in range(read))
        lib=Library();bus=Bus()
        with patch('car.bme.C.CDLL',return_value=lib):sensor=BME688(bus,'unused.so')
        buf=(C.c_uint8*49)()
        self.assertEqual(lib.callbacks[0](0x80,buf,49,None),0)
        self.assertEqual(bytes(buf),bytes(range(0x80,0x80+49)))
        self.assertEqual([x[3] for x in bus.calls],[32,17])
        interleaved=(C.c_uint8*3)(0x42,0x75,0xa0)
        self.assertEqual(lib.callbacks[1](0x74,interleaved,3,None),0)
        self.assertEqual(bus.calls[-1],(0x77,0x74,b'\x42\x75\xa0',0))
        self.assertEqual(sensor.sample(),([2550,50000,101325,100000],7))

if __name__=='__main__':unittest.main()
