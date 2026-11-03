"""Independent gas-confirmation protocol; no motion commands or keepalives."""
import struct
from .protocol import crc16

SOURCE_CLEARED=0
SOURCE_CONFIRMED=1
QUERY=2

def event_packet(seq,boot_id,issued_ms,event,ttl_ms=500):
    if event not in (0,1,2):raise ValueError('Unknown source event')
    if not 100<=ttl_ms<=500:raise ValueError('TTL must be 100..500ms')
    body=struct.pack('<2sBBIIIBBHiiH',b'\xa5\x5a',1,2,seq&0xffffffff,
                     boot_id&0xffffffff,issued_ms&0xffffffff,event,0,ttl_ms,0,0,0)
    return body+struct.pack('<H',crc16(body))

def decode_ack(packet):
    if len(packet)!=192 or packet[:4]!=b'\xa5\x5a\x01\x83':raise ValueError('Invalid source ACK')
    if packet[188:190]!=b'\0\0' or crc16(packet[:-2])!=int.from_bytes(packet[-2:],'little'):raise ValueError('Invalid source ACK CRC/reserved')
    w=struct.unpack_from('<45I',packet,8)
    if w[3]>5 or w[5]>1 or w[7]!=250 or w[8]!=0x4c454431 or any(w[9:]):raise ValueError('Unknown source ACK schema')
    return dict(boot_id=w[0],timestamp_ms=w[1],request_seq=w[2],status=w[3],
                event=w[4],confirmed=bool(w[5]),last_event_seq=w[6],blink_half_period_ms=w[7],
                ack_seq=int.from_bytes(packet[4:8],'little'))

class SourceIndicator:
    """Call poll() from the existing sole IO loop; never owns the socket."""
    def __init__(self,client):
        self.client=client;self.pending=None;self.boot=None;self.sequence=0
        self.last_tx=0;self.started=0;self.result=None

    def request(self,event):
        import time
        from .protocol import signed32
        now=time.monotonic();s=self.client.latest
        if self.pending is not None:raise RuntimeError('Source event already pending')
        if not s or now-self.client.received_at>.15:raise RuntimeError('Fresh telemetry required')
        if not s.get('source_led_supported'):raise RuntimeError('LED firmware not installed')
        ack=self.client.latest_source_ack
        if ack and ack['boot_id']==s['boot_id']:
            if signed32(ack['last_event_seq']-self.sequence)>0:self.sequence=ack['last_event_seq']
        self.sequence=(self.sequence+1)&0xffffffff;self.boot=s['boot_id']
        issued=(s['timestamp_ms']+int((now-self.client.received_at)*1000)-20)&0xffffffff
        self.pending=event_packet(self.sequence,self.boot,issued,event)
        self.started=now;self.last_tx=now;self.result=None;self.client.latest_source_ack=None
        self.client.sock.sendto(self.pending,self.client.address)

    def poll(self):
        import time
        if self.pending is None:return self.result
        now=time.monotonic();s=self.client.latest;ack=self.client.latest_source_ack
        if s and s['boot_id']!=self.boot:
            self.pending=None;raise RuntimeError('Board restarted; workflow must explicitly resynchronize')
        if ack and ack['boot_id']==self.boot and ack['request_seq']==self.sequence:
            self.pending=None;self.result=ack
            if ack['status']:raise RuntimeError('Source event rejected: '+str(ack['status']))
            return ack
        if now-self.started>.8:
            self.pending=None;raise TimeoutError('Source event ACK timeout; state unknown, query after recovery')
        if now-self.last_tx>=.1:
            self.client.sock.sendto(self.pending,self.client.address);self.last_tx=now
        return None
