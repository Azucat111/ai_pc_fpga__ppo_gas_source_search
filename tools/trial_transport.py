"""UDP transport with the same telemetry counters as the commissioning UART adapter."""
import socket
import time

from check_car_uart import SerialSocket
from car.protocol import crc16


class UDPTransport:
    def __init__(self, address, port=4242):
        self.peer = (address, port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.connect(self.peer)
        self.socket.settimeout(.2)
        self.frames = self.crc_errors = self.discarded = 0
        self.command_log = []

    def settimeout(self, timeout):
        self.socket.settimeout(timeout)

    def sendto(self, packet, address):
        if address != self.peer:
            raise ValueError('Unexpected UDP trial destination')
        if len(packet)==32:
            self.command_log.append(dict(host_monotonic_s=time.monotonic(),
                                         seq=int.from_bytes(packet[4:8],'little'),
                                         issued_ms=int.from_bytes(packet[12:16],'little'),mode=packet[16]))
        return self.socket.send(packet)

    def recvfrom(self, length):
        packet, source = self.socket.recvfrom(length)
        if len(packet) == 192 and packet[:4] == b'\xa5\x5a\x01\x81':
            if crc16(packet[:-2]) == int.from_bytes(packet[-2:], 'little'):
                self.frames += 1
            else:
                self.crc_errors += 1
        else:
            self.discarded += len(packet)
        return packet, source

    def close(self):
        self.socket.close()


def open_transport(port=None, ip=None):
    if bool(port) == bool(ip):
        raise ValueError('Choose exactly one UART port or UDP IP address')
    if port:
        return SerialSocket(port), 'serial', 0
    return UDPTransport(ip), ip, 4242
