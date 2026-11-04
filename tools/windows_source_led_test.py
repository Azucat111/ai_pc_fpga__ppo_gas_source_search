"""Operator-run LED check with Wi-Fi restoration; rejects every motion packet."""
import argparse,json,socket,sys,time,uuid
from datetime import datetime
from pathlib import Path
from offline_ground_trial_windows import CarWiFi,ROOT,IP
from car.config import load_config
from car.host import Client
from car.source_event import SourceIndicator,QUERY,SOURCE_CONFIRMED,SOURCE_CLEARED


class LEDOnlySocket:
    def __init__(self,inner=None):
        self.inner=inner if inner is not None else socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.event_packets=0;self.registrations=0
    def sendto(self,data,address):
        if data==b'FCAR?':self.registrations+=1
        elif len(data)==32 and data[:4]==b'\xa5\x5a\x01\x02':self.event_packets+=1
        else:raise RuntimeError('LED-only transport blocked a non-event packet')
        return self.inner.sendto(data,address)
    def __getattr__(self,name):return getattr(self.inner,name)


def run(directory):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=False)
    wifi=None;client=None;transport=None
    result=dict(restored=False,car_motion_commands_sent=0,car_stop_commands_sent=0,physical_observation_pending=True)
    def record(kind,**fields):
        item=dict(kind=kind,pc_time_ns=time.time_ns(),pc_monotonic_ns=time.monotonic_ns(),**fields)
        with (directory/'events.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(item)+'\n')
    def log(message):
        print(message,flush=True)
        with (directory/'console.log').open('a',encoding='utf-8') as f:f.write(message+'\n')
    def connect_client():
        nonlocal transport
        transport=LEDOnlySocket();c=Client(IP,load_config(ROOT/'config/car.json'),sock=transport)
        try:
            s=c.receive(5)
            if not s.get('source_led_supported'):raise RuntimeError('LED capability missing')
            if s['last_action']!='stop':raise RuntimeError('Wait until the current motion finishes before the LED check')
            result.setdefault('boot_id',s['boot_id'])
            if s['boot_id']!=result['boot_id']:raise RuntimeError('Board restarted during LED test')
            record('state',state=s)
            return c
        except BaseException:c.close();raise
    def send(engine,event):
        engine.request(event)
        record('request',boot_id=engine.boot,seq=engine.sequence,event=event,packet_hex=engine.pending.hex())
        while True:
            try:client.receive(.02)
            except TimeoutError:pass
            ack=engine.poll()
            if ack is not None:
                record('ack',ack=ack);return ack
    try:
        wifi=CarWiFi();log('只测试板载LED，不发运动、STOP或保活。正在连接小车Wi-Fi，结束后恢复原网络。')
        wifi.connect();client=connect_client();engine=SourceIndicator(client)
        result['initial']=send(engine,QUERY)
        if result['initial']['confirmed']:raise RuntimeError('Initial LED already confirmed; refusing to overwrite existing indication')
        result['confirmed_ack']=send(engine,SOURCE_CONFIRMED)
        if not result['confirmed_ack']['confirmed']:raise RuntimeError('Confirmation did not latch')
        log('请看PYNQ板上的LD5蓝灯：现在应开始闪烁，约12秒后自动熄灭。')
        # Close the event client; no heartbeat or further event is sent for 12s.
        result['first_event_packets']=transport.event_packets
        client.close();client=None
        time.sleep(12)
        client=connect_client();engine=SourceIndicator(client)
        result['latched_after_silence']=send(engine,QUERY)
        if not result['latched_after_silence']['confirmed']:raise RuntimeError('LED state did not persist without event client')
        result['cleared_ack']=send(engine,SOURCE_CLEARED)
        if result['cleared_ack']['confirmed']:raise RuntimeError('Clear did not latch')
        log('已收到清除ACK：LED现在应熄灭。')
        result['final']=send(engine,QUERY)
        result['protocol_passed']=not result['final']['confirmed']
        time.sleep(2)
    except (OSError,RuntimeError,ValueError,TimeoutError,KeyboardInterrupt) as exc:
        result['error']=type(exc).__name__+': '+str(exc);log('检查未完成：'+result['error'])
        log('若确认消息已经送达，LED可能仍闪烁；不发送运动或STOP，恢复网络后保留日志排查。')
    finally:
        if transport:result['last_event_packets']=transport.event_packets
        if client:client.close()
        if wifi:
            try:
                log('正在恢复原Wi-Fi：'+wifi.restore_profile);result['restored']=wifi.restore()
            except Exception as exc:result['restore_error']=str(exc)
        result['ok']=bool(result.get('protocol_passed') and result['restored'] and not result.get('error'))
        (directory/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        log('协议检查：'+str(result.get('protocol_passed',False))+'；原Wi-Fi恢复：'+str(result['restored']))
        log('结果：'+str(directory))
        if not result['restored']:log('请手动连接回原Wi-Fi，再返回对话。')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dry-run',action='store_true');args=p.parse_args()
    if args.dry_run:
        print('DRY RUN: FCAR? and type02 events only; QUERY -> CONFIRMED -> close socket12s -> QUERY -> CLEARED -> QUERY -> restore original Wi-Fi. No networking or motion in dry run.');return 0
    directory=ROOT/'build/output'/('local_led_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
    return 0 if run(directory)['ok'] else 1


if __name__=='__main__':raise SystemExit(main())
