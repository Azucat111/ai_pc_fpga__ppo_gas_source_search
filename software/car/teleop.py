"""Nonblocking dead-man remote control, using the existing PL action lease."""
import time
from .config import rpm_target
from .host import ControlBusy
from .protocol import signed32

STRAIGHT_RPM=40
TURN_RPM=20


class Teleop:
    def __init__(self,client):
        self.client=client;self.phase='idle';self.action=None;self.boot=None
        self.pending={};self.next_heartbeat=0;self.stop_packet=None;self.stop_sent=0

    @property
    def owns(self):return self.phase!='idle'

    def abandon(self):
        self.phase='idle';self.action=None;self.pending.clear();self.stop_packet=None

    def transmit(self,mode,p0=0,p1=0):
        seq,boot,packet=self.client._make_packet(mode,p0,p1,refresh=False)
        self.client.sock.sendto(packet,self.client.address)
        return seq,boot,packet

    def start(self,direction):
        s=self.client.latest;now=time.monotonic()
        if self.owns:raise RuntimeError('请先松开按键，等待停车')
        if not s or now-self.client.received_at>.15:raise RuntimeError('等待新鲜的小车数据')
        if s['last_action']!='stop':raise ControlBusy('另一台正在控制；等待停车后重新按键')
        if not s['enabled'] or not s['geometry_valid']:raise RuntimeError('小车尚未准备好')
        if s['imu_gyro_z'] is None or s['imu_age_ms']>150:raise RuntimeError('IMU数据未就绪')
        if not s.get('straight_gyro_heading'):raise RuntimeError('小车需要v0.1.8运动程序')
        # Forward/backward retain the validated gyro straight-heading controller.
        mode,left,right={'w':(2,STRAIGHT_RPM,STRAIGHT_RPM),
                         's':(2,-STRAIGHT_RPM,-STRAIGHT_RPM),
                         'a':(1,-TURN_RPM,TURN_RPM),'d':(1,TURN_RPM,-TURN_RPM)}[direction]
        seq,self.boot,_=self.transmit(mode,rpm_target(left,self.client.config,'left'),
                                    rpm_target(right,self.client.config,'right'))
        self.action=seq;self.pending={seq:now};self.phase='starting'
        self.next_heartbeat=now

    def stop(self,force=False):
        if not self.owns and not force:return
        if self.client.latest is None:return
        seq,self.boot,packet=self.transmit(0)
        now=time.monotonic();self.phase='stopping';self.action=seq
        self.pending={seq:now};self.stop_packet=packet;self.stop_sent=now

    def poll(self,timeout=.02):
        try:s=self.client.receive(timeout=timeout)
        except TimeoutError:s=None
        except ControlBusy:
            # A refused requester must not send STOP to the actual controller.
            self.abandon();raise
        if s and self.owns:
            if s['boot_id']!=self.boot:
                self.abandon();raise RuntimeError('小车已重启，松开后重新按键')
            acknowledged=signed32(s['last_command_seq']-self.action)>=0
            if self.phase=='stopping':
                if s['last_action']=='stop' and acknowledged:self.abandon()
            else:
                if s['last_checked_command_seq'] in self.pending and s['command_status']:
                    raise RuntimeError('小车拒绝指令：'+str(s['command_status']))
                if acknowledged:
                    if s['last_action']=='stop':raise RuntimeError('小车已停车：'+s['stop_reason'])
                    if s['action_id']!=self.action:
                        self.abandon();raise ControlBusy('控制动作已被替换，请重新按键')
                    self.phase='driving'
                for seq in list(self.pending):
                    if signed32(s['last_command_seq']-seq)>=0:del self.pending[seq]
        now=time.monotonic()
        if self.owns:
            if now-self.client.received_at>.4 or (self.pending and now-min(self.pending.values())>.4):
                raise TimeoutError('通信或确认超时，请松开按键')
            if self.phase=='driving' and now>=self.next_heartbeat:
                seq,_,_=self.transmit(4,signed32(self.action))
                self.pending[seq]=now;self.next_heartbeat=now+.1
            elif self.phase=='stopping' and now-self.stop_sent>=.1:
                self.client.sock.sendto(self.stop_packet,self.client.address);self.stop_sent=now
        return s
