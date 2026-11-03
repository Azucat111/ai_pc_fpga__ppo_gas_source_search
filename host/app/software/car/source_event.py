"""Independent PYNQ source-indicator events; never encode motion commands."""
import struct
import time

from .protocol import crc16, signed32

SOURCE_CLEARED = 0
SOURCE_CONFIRMED = 1
QUERY = 2


def event_packet(seq, boot_id, issued_ms, event, ttl_ms=500):
    if event not in (SOURCE_CLEARED, SOURCE_CONFIRMED, QUERY):
        raise ValueError('Unknown source event')
    if not 100 <= ttl_ms <= 500:
        raise ValueError('TTL must be 100..500ms')
    body = struct.pack('<2sBBIIIBBHiiH', b'\xa5\x5a', 1, 2,
                       seq & 0xffffffff, boot_id & 0xffffffff,
                       issued_ms & 0xffffffff, event, 0, ttl_ms, 0, 0, 0)
    return body + struct.pack('<H', crc16(body))


def decode_ack(packet):
    if len(packet) != 192 or packet[:4] != b'\xa5\x5a\x01\x83':
        raise ValueError('Invalid source ACK')
    if packet[188:190] != b'\0\0' or crc16(packet[:-2]) != int.from_bytes(packet[-2:], 'little'):
        raise ValueError('Invalid source ACK CRC/reserved')
    words = struct.unpack_from('<45I', packet, 8)
    if (words[3] > 5 or words[5] > 1 or words[7] != 250 or
            words[8] != 0x4c454431 or any(words[9:])):
        raise ValueError('Unknown source ACK schema')
    return {
        'boot_id': words[0], 'timestamp_ms': words[1],
        'request_seq': words[2], 'status': words[3], 'event': words[4],
        'confirmed': bool(words[5]), 'last_event_seq': words[6],
        'blink_half_period_ms': words[7],
        'ack_seq': int.from_bytes(packet[4:8], 'little'),
    }


class SourceIndicator:
    """Nonblocking event sender using the car Client's existing socket."""

    def __init__(self, client):
        self.client = client
        self.pending = None
        self.boot = None
        self.sequence = 0
        self.last_tx = 0.0
        self.started = 0.0
        self.result = None

    def request(self, event):
        now = time.monotonic()
        telemetry = self.client.latest
        if self.pending is not None:
            raise RuntimeError('Source event already pending')
        if not telemetry or now - self.client.received_at > 0.15:
            raise RuntimeError('Fresh telemetry required')
        if not telemetry.get('source_led_supported'):
            raise RuntimeError('LED firmware not installed')
        ack = self.client.latest_source_ack
        if ack and ack['boot_id'] == telemetry['boot_id']:
            if signed32(ack['last_event_seq'] - self.sequence) > 0:
                self.sequence = ack['last_event_seq']
        self.sequence = (self.sequence + 1) & 0xffffffff
        self.boot = telemetry['boot_id']
        issued = (telemetry['timestamp_ms'] +
                  int((now - self.client.received_at) * 1000) - 20) & 0xffffffff
        self.pending = event_packet(self.sequence, self.boot, issued, event)
        self.started = now
        self.last_tx = now
        self.result = None
        self.client.latest_source_ack = None
        self.client.sock.sendto(self.pending, self.client.address)

    def poll(self):
        if self.pending is None:
            return self.result
        now = time.monotonic()
        telemetry = self.client.latest
        ack = self.client.latest_source_ack
        if telemetry and telemetry['boot_id'] != self.boot:
            self.pending = None
            raise RuntimeError('Board restarted; workflow must explicitly resynchronize')
        if ack and ack['boot_id'] == self.boot and ack['request_seq'] == self.sequence:
            self.pending = None
            self.result = ack
            if ack['status']:
                raise RuntimeError(f"Source event rejected: {ack['status']}")
            return ack
        if now - self.started > 0.8:
            self.pending = None
            raise TimeoutError('Source event ACK timeout; state unknown, query after recovery')
        if now - self.last_tx >= 0.1:
            self.client.sock.sendto(self.pending, self.client.address)
            self.last_tx = now
        return None
