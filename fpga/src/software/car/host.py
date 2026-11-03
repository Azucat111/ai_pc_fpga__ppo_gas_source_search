"""Ubuntu CLI. All closed-loop control remains in the FPGA."""
import socket
import time
from .config import load_config,rpm_target,velocity_targets
from .protocol import command,decode,signed32
from .gas10 import decode_gas10
from .source_event import decode_ack

class ControlBusy(RuntimeError):
    """A different station owns the current action; do not stop its action."""

class Client:
    def __init__(self,address,config,port=4242,sock=None):
        self.address=(address,port);self.config=config
        self.sock=sock or socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sock.settimeout(0.2);self.latest=None;self.received_at=0;self.sequence=None;self.latest_gas10=None;self.gas10_received_at=0;self.latest_source_ack=None
        self.sock.sendto(b'FCAR?',self.address);self.last_hello=time.monotonic()
    def receive(self,timeout=2):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if time.monotonic()-self.last_hello>5:
                self.sock.sendto(b'FCAR?',self.address);self.last_hello=time.monotonic()
            self.sock.settimeout(min(.2,max(.001,deadline-time.monotonic())))
            try:packet,source=self.sock.recvfrom(2048)
            except socket.timeout:continue
            if source!=self.address:continue
            if packet==b'FCAR_BUSY':raise ControlBusy('Another station is controlling the car; wait until it stops')
            if packet==b'FCAR_FULL':raise ControlBusy('Two clients are already registered; close an old client and wait 60 seconds')
            if packet==b'FCAR_NOT_READY':raise RuntimeError('Car is not ready for control; wait for fresh STOP telemetry and retry')
            if packet[:4]==b'\xa5\x5a\x01\x83':
                try:
                    ack=decode_ack(packet)
                    if self.latest is not None and ack['boot_id']==self.latest['boot_id']:
                        self.latest_source_ack=ack
                except ValueError:pass
                continue
            if packet[:4]==b'\xa5\x5a\x01\x82':
                try:
                    gas=decode_gas10(packet)
                    if (self.latest is None or gas['boot_id']==self.latest['boot_id']) and (self.latest_gas10 is None or gas['boot_id']!=self.latest_gas10['boot_id'] or signed32(gas['seq']-self.latest_gas10['seq'])>0):
                        self.latest_gas10=gas;self.gas10_received_at=time.monotonic()
                except ValueError:pass
                continue
            try:item=decode(packet,self.config)
            except ValueError:continue
            if self.latest and item['boot_id']==self.latest['boot_id'] and signed32(item['seq']-self.latest['seq'])<=0:
                continue
            if self.latest and item['boot_id']!=self.latest['boot_id']:
                self.latest_gas10=None;self.gas10_received_at=0;self.latest_source_ack=None
            self.latest=item;self.received_at=time.monotonic();return item
        raise TimeoutError('No valid telemetry; check Wi-Fi, ESP UART and board service')
    def _make_packet(self,mode,p0=0,p1=0,refresh=True):
        if self.latest is None:self.receive()
        # STOP can use the last known boot ID: the PL ignores its age/TTL.
        # Never delay this best-effort stop waiting for a broken return link.
        if mode and refresh and time.monotonic()-self.received_at>0.4:self.receive()
        boot=self.latest['boot_id']
        if not boot:raise RuntimeError('Run car-board to configure the FPGA first')
        if mode and not self.latest['enabled']:raise RuntimeError('Board motion is disabled')
        previous=self.latest['last_command_seq']
        if self.sequence is not None and signed32(self.sequence-previous)>0:previous=self.sequence
        self.sequence=(previous+1)&0xffffffff
        age=int((time.monotonic()-self.received_at)*1000)
        issued=(self.latest['timestamp_ms']+age-20)&0xffffffff
        packet=command(self.sequence,boot,issued,mode,p0,p1)
        return self.sequence,boot,packet
    def send(self,mode,p0=0,p1=0):
        sequence,boot,packet=self._make_packet(mode,p0,p1)
        deadline=time.monotonic()+1.0;last_tx=-1.0
        while time.monotonic()<deadline:
            if time.monotonic()-last_tx>0.1:
                self.sock.sendto(packet,self.address);last_tx=time.monotonic()
            try:s=self.receive(timeout=0.2)
            except TimeoutError:continue
            if s['boot_id']!=boot:raise RuntimeError('FPGA restarted; motion has been cancelled')
            if s['last_command_seq']==sequence:return s
            if s['last_checked_command_seq']==sequence and s['command_status']:
                raise RuntimeError(f"FPGA rejected command, status={s['command_status']}")
        raise TimeoutError('Command acknowledgement timeout')
    def hold(self,mode,p0,p1,duration):
        # Include the initial command's acknowledgement time in the duration.
        deadline=time.monotonic()+duration
        busy=False
        try:
            started=self.send(mode,p0,p1);action=started['action_id'];boot=started['boot_id']
            if started['last_action']=='stop':
                if mode==3 and started['stop_reason']=='turn_complete':return started
                raise RuntimeError('Motion stopped: '+started['stop_reason'])
            next_heartbeat=time.monotonic();pending={}
            while time.monotonic()<deadline:
                now=time.monotonic()
                if now-self.received_at>.45:
                    raise TimeoutError('Telemetry stopped during motion')
                if pending and now-min(pending.values())>.45:
                    raise TimeoutError('Heartbeat acknowledgement timed out')
                # Do not serialize the next keepalive behind an acknowledgement.
                # A later cumulative ACK covers earlier lost heartbeats too.
                if now>=next_heartbeat:
                    seq,_,packet=self._make_packet(4,signed32(action),refresh=False)
                    self.sock.sendto(packet,self.address);pending[seq]=now
                    next_heartbeat=now+.1
                wait=min(.05,max(.001,next_heartbeat-time.monotonic()),max(.001,deadline-time.monotonic()))
                try:s=self.receive(timeout=wait)
                except TimeoutError:continue
                if s['boot_id']!=boot:raise RuntimeError('FPGA restarted')
                if s['last_action']=='stop':
                    if mode==3 and s['stop_reason']=='turn_complete':return s
                    raise RuntimeError('Motion stopped: '+s['stop_reason'])
                if s['action_id']!=action:raise RuntimeError('Another action replaced this command')
                if s['last_checked_command_seq'] in pending and s['command_status']:
                    raise RuntimeError(f"FPGA rejected heartbeat, status={s['command_status']}")
                for seq in list(pending):
                    if signed32(s['last_command_seq']-seq)>=0:del pending[seq]
            if mode==3:raise TimeoutError('Turn did not settle within --duration')
            return self.latest
        except ControlBusy:
            busy=True
            raise
        finally:
            if not busy:
                try:self.send(0)
                except (OSError,TimeoutError,RuntimeError):pass
    def close(self):self.sock.close()

