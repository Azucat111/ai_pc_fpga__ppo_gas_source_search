"""The actual UDP commissioning adapter preserves commands and detects corrupt telemetry."""
from pathlib import Path
import socket
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from trial_transport import UDPTransport, open_transport
from car.protocol import crc16


class TrialUDPTests(unittest.TestCase):
    def test_local_datagrams_and_bad_checksum(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server:
            server.bind(('127.0.0.1', 0));server.settimeout(1)
            transport = UDPTransport(*server.getsockname())
            try:
                transport.sendto(b'FCAR?', server.getsockname())
                data, peer = server.recvfrom(512)
                self.assertEqual(data, b'FCAR?')
                body = b'\xa5\x5a\x01\x81'+bytes(186)
                packet = body+crc16(body).to_bytes(2, 'little')
                for payload in (packet, packet[:-1]+bytes([packet[-1]^1]), b'bad'):
                    server.sendto(payload, peer)
                    received, source = transport.recvfrom(2048)
                    self.assertEqual((received, source), (payload, server.getsockname()))
                self.assertEqual((transport.frames, transport.crc_errors, transport.discarded), (1, 1, 3))
                with self.assertRaises(ValueError):
                    transport.sendto(b'no', ('127.0.0.1', 1))
            finally:
                transport.close()

    def test_choose_exactly_one_link(self):
        for args in ({}, dict(port='COM13', ip='192.168.4.1')):
            with self.assertRaises(ValueError):
                open_transport(**args)
