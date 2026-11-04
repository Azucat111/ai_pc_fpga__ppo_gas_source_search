"""User-run source indicator check; never sends motion, STOP or keepalives."""
import argparse,json,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'software'))
from car.host import Client
from car.config import load_config
from car.source_event import SourceIndicator,QUERY,SOURCE_CONFIRMED,SOURCE_CLEARED

def main():
    p=argparse.ArgumentParser()
    p.add_argument('event',choices=('query','confirm','clear'))
    p.add_argument('--ip',default='192.168.4.1')
    p.add_argument('--log',default='build/source_led_check.jsonl')
    args=p.parse_args();target=Path(args.log);target.parent.mkdir(parents=True,exist_ok=True)
    client=Client(args.ip,load_config(ROOT/'config/car.json'));engine=SourceIndicator(client)
    def record(data):
        data.update(pc_time_ns=time.time_ns(),pc_monotonic_ns=time.monotonic_ns())
        with target.open('a',encoding='utf-8') as log:log.write(json.dumps(data)+'\n')
        print(json.dumps(data),flush=True)
    def send(event):
        engine.request(event);record(dict(kind='request',boot_id=engine.boot,seq=engine.sequence,event=event,packet_hex=engine.pending.hex()))
        while True:
            try:client.receive(.02)
            except TimeoutError:pass
            ack=engine.poll()
            if ack is not None:record(dict(kind='ack',ack=ack));return ack
    try:
        client.receive();send(QUERY)
        if args.event!='query':send(SOURCE_CONFIRMED if args.event=='confirm' else SOURCE_CLEARED)
    except (OSError,RuntimeError,TimeoutError) as exc:
        record(dict(kind='error',error=str(exc),latest_ack=client.latest_source_ack));raise SystemExit(1)
    finally:client.close() # Observers must not STOP another controller.

if __name__=='__main__':main()
