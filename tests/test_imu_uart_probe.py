import struct
import sys
from pathlib import Path
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from check_imu_uart import request,take_frames


class ImuUartProbeTests(unittest.TestCase):
    def test_request_matches_vendor_protocol_and_forbids_configuration(self):
        self.assertEqual(request(1),bytes.fromhex('7e230780010029'))
        for function in (0x61,0x70,0x71,0xa0):
            with self.assertRaises(ValueError):request(function)

    def test_split_frame_and_checksum_recovery(self):
        payload=struct.pack('<3f',.1,.2,-1.5)
        body=bytes((0x7e,0x23,len(payload)+5,0x26))+payload
        frame=body+bytes((sum(body)&255,))
        b=bytearray(b'noise'+frame[:7])
        self.assertEqual(take_frames(b),([],0))
        b.extend(frame[7:])
        self.assertEqual(take_frames(b),([(0x26,payload)],0))
        b.extend(frame[:-1]+bytes((frame[-1]^1,))+frame)
        frames,bad=take_frames(b)
        self.assertEqual(frames,[(0x26,payload)])
        self.assertEqual(bad,1)
