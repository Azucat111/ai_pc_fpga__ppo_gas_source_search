"""Passive UDP telemetry audit for Ubuntu/Windows; sends FCAR? registration only."""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import socket
import statistics
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'software'))
from car.config import load_config
from car.protocol import decode,signed32,crc16
from car.gas10 import decode_gas10
from car.source_event import decode_ack

REPLIES={b'FCAR_BUSY',b'FCAR_FULL',b'FCAR_NOT_READY'}


class Audit:
    def __init__(self,config):
        self.source_event_acks=0;self.latest_source_ack=None
        self.config=config;self.valid_frames=0;self.unique_frames=0
        self.invalid_datagrams=0;self.crc_errors=0;self.duplicates_or_old=0
        self.router_replies={};self.boots={};self.intervals=[];self.last_receive_ns=None;self.gas_usable_frames=0;self.gas10_frames=0;self.gas10_valid=0;self.gas10_seen=set();self.latest_gas10=None
        self.validity={k:0 for k in ('imu','bme','tgs0','tgs1')}
        self.max_age_ms={k:0 for k in self.validity}
        self.updates={k:set() for k in self.validity}

    def observe(self,packet,receive_ns):
        if packet in REPLIES:
            key=packet.decode('ascii');self.router_replies[key]=self.router_replies.get(key,0)+1
            return dict(kind='router_reply',reply=key)
        if packet[:4]==b'\xa5\x5a\x01\x83':
            try:
                sample=decode_ack(packet);self.source_event_acks+=1;self.latest_source_ack=sample
                return dict(kind='source_ack',crc_verified=True,source_ack=sample)
            except ValueError as exc:
                self.invalid_datagrams+=1
                if len(packet)==192 and crc16(packet[:-2])!=int.from_bytes(packet[-2:],'little'):self.crc_errors+=1
                return dict(kind='invalid_source_ack',error=str(exc))
        if packet[:4]==b'\xa5\x5a\x01\x82':
            try:
                sample=decode_gas10(packet)
                self.gas10_frames+=1;self.latest_gas10=sample
                key=(sample['boot_id'],sample['scan_id'])
                if key not in self.gas10_seen:
                    self.gas10_seen.add(key)
                    if sample['acquisition_valid']:self.gas10_valid+=1
                return dict(kind='gas10',crc_verified=True,gas10=sample)
            except ValueError as exc:
                self.invalid_datagrams+=1
                if len(packet)==192 and crc16(packet[:-2])!=int.from_bytes(packet[-2:],'little'):self.crc_errors+=1
                return dict(kind='invalid_gas10',error=str(exc))
        try:item=decode(packet,self.config)
        except ValueError as exc:
            self.invalid_datagrams+=1
            if len(packet)==192 and packet[:4]==b'\xa5\x5a\x01\x81' and crc16(packet[:-2])!=int.from_bytes(packet[-2:],'little'):
                self.crc_errors+=1
            return dict(kind='invalid',error=str(exc))
        self.valid_frames+=1;boot=item['boot_id'];previous=self.boots.get(boot)
        delta=signed32(item['seq']-previous['seq']) if previous else 1
        record=dict(kind='telemetry',crc_verified=True,crc_wire=int.from_bytes(packet[-2:],'little'),
                    frame_bytes=len(packet),state_payload_bytes=180,accepted_for_metrics=delta>0,telemetry=item)
        if delta<=0:
            self.duplicates_or_old+=1;return record
        if previous:
            dt=(item['timestamp_ms']-previous['raw_ms'])&0xffffffff
            unwrapped=previous['unwrapped_ms']+dt
            previous['missing_frames']+=delta-1
        else:
            unwrapped=item['timestamp_ms']
            previous=dict(first_seq=item['seq'],first_unwrapped_ms=unwrapped,missing_frames=0,
                          frames=0,first_errors=item['errors'])
            self.boots[boot]=previous
        previous.update(seq=item['seq'],raw_ms=item['timestamp_ms'],unwrapped_ms=unwrapped,
                        last_errors=item['errors'],frames=previous['frames']+1)
        self.unique_frames+=1
        if item['bme688']['gas_resistance_ohm'] is not None:self.gas_usable_frames+=1
        if self.last_receive_ns is not None:self.intervals.append((receive_ns-self.last_receive_ns)/1e6)
        self.last_receive_ns=receive_ns
        record['board_timestamp_us_derived_from_ms']=unwrapped*1000
        record['board_time_resolution_us']=1000
        # Acquisition/measurement start is not available; these are reported commit times.
        streams={'imu':(item['imu_yaw'] is not None,item['imu_age_ms']),
                 'bme':(item['bme688']['valid'],item['bme688']['age_ms']),
                 'tgs0':(item['tgs2602'][0]['valid'],item['tgs2602'][0]['age_ms']),
                 'tgs1':(item['tgs2602'][1]['valid'],item['tgs2602'][1]['age_ms'])}
        record['reported_sample_commit_us_derived']={}
        for key,(valid,age) in streams.items():
            self.max_age_ms[key]=max(self.max_age_ms[key],age)
            timestamp=(unwrapped-age)*1000 if valid else None
            record['reported_sample_commit_us_derived'][key]=timestamp
            if valid:
                self.validity[key]+=1;self.updates[key].add((boot,timestamp))
        return record

    def summary(self):
        boots=[]
        for boot,state in self.boots.items():
            span=state['unwrapped_ms']-state['first_unwrapped_ms']
            boots.append(dict(boot_id=boot,frames=state['frames'],estimated_missing_telemetry_frames=state['missing_frames'],
                              board_span_ms=span,received_frame_rate_hz=(state['frames']-1)*1000/span if span>0 else None,
                              error_counter_delta={k:(state['last_errors'][k]-v)&0xffffffff for k,v in state['first_errors'].items()}))
        return dict(source_event_acks=self.source_event_acks,latest_source_ack=self.latest_source_ack,gas10_frames=self.gas10_frames,gas10_valid_unique_scans=self.gas10_valid,latest_gas10=self.latest_gas10,validation_scope='passive_transport_and_schema_observation_only',requirements_complete=False,
                    unique_frames=self.unique_frames,valid_frames_including_duplicates=self.valid_frames,
                    invalid_datagrams=self.invalid_datagrams,crc_errors=self.crc_errors,
                    duplicates_or_old=self.duplicates_or_old,router_replies=self.router_replies,boots=boots,
                    fresh_valid_frame_count=self.validity,bme_gas_usable_frames=self.gas_usable_frames,max_reported_age_ms=self.max_age_ms,
                    observed_updates_by_reconstructed_commit_time={k:len(v) for k,v in self.updates.items()},
                    pc_interarrival_ms=dict(mean=statistics.mean(self.intervals) if self.intervals else None,
                                            std=statistics.pstdev(self.intervals) if self.intervals else None,
                                            max=max(self.intervals) if self.intervals else None),
                    time_note='Board us values are ms*1000, not true microsecond precision; PC and board clocks are not synchronized. Gaps estimate missing telemetry, not ADC sample loss. Commit timestamps do not identify BME heater-step acquisition times.')


def collect(ip,port,config,seconds,output,sock=None,require_gas10=False):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    audit=Audit(config);registrations=0;error=None;interrupted=False
    connection=sock or socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    try:
        connection.connect((ip,port));end=time.monotonic()+seconds;next_hello=0
        with (output/'raw_datagrams.jsonl').open('w',encoding='utf-8') as raw, (output/'decoded_frames.jsonl').open('w',encoding='utf-8') as decoded:
            while time.monotonic()<end:
                now=time.monotonic()
                if now>=next_hello:
                    connection.send(b'FCAR?');registrations+=1;next_hello=now+5
                connection.settimeout(min(.2,max(.001,end-time.monotonic())))
                try:packet=connection.recv(65535)
                except socket.timeout:continue
                mono_ns=time.monotonic_ns();utc_us=time.time_ns()//1000
                clocks=dict(pc_receive_timestamp_us=utc_us,pc_receive_monotonic_us=mono_ns//1000)
                raw.write(json.dumps(dict(clocks,packet_hex=packet.hex(),packet_length=len(packet)))+'\n')
                record=audit.observe(packet,mono_ns)
                decoded.write(json.dumps(dict(clocks,**record),ensure_ascii=False,allow_nan=False)+'\n')
    except KeyboardInterrupt:interrupted=True
    except OSError as exc:error=type(exc).__name__+': '+str(exc)
    finally:
        connection.close()
        result=audit.summary()
        result.update(peer=dict(ip=ip,port=port),requested_seconds=seconds,registration_packets_sent=registrations,
                      motion_commands_sent=0,stop_commands_sent=0,interrupted=interrupted,error=error,
                      observer_sends_only_ascii_registration=True)
        result['transport_check_passed']=(audit.unique_frames>=2 and any(boot!=0 for boot in audit.boots)
                                          and not error and not audit.invalid_datagrams and not audit.router_replies)
        result['telemetry_transport_check_passed']=result['transport_check_passed']
        result['gas10_frame_transport_check_passed']=bool(audit.gas10_frames and not error and not audit.invalid_datagrams and not audit.router_replies)
        result['gas10_acquisition_check_passed']=audit.gas10_valid>=3
        result['gas10_required']=require_gas10
        result['capture_check_passed']=bool(result['transport_check_passed'] and (not require_gas10 or result['gas10_acquisition_check_passed']) and not interrupted)
        (output/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ip',default='192.168.4.1');parser.add_argument('--port',type=int,default=4242)
    parser.add_argument('--seconds',type=float,default=30)
    parser.add_argument('--require-gas10',action='store_true',help='Require at least three complete valid GAS10 scans as well as telemetry transport')
    parser.add_argument('--output',type=Path,default=ROOT/'reports'/('ubuntu_link_'+datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%SZ')))
    args=parser.parse_args()
    if not 1<=args.seconds<=600:parser.error('--seconds must be1..600')
    if not 1<=args.port<=65535:parser.error('Invalid UDP port')
    if args.output.exists():parser.error('Use a fresh output path; existing logs are never overwritten')
    result=collect(args.ip,args.port,load_config(ROOT/'config/car.json'),args.seconds,args.output,require_gas10=args.require_gas10)
    print(json.dumps(result,ensure_ascii=False,indent=2))
    print('Results:',args.output)
    print('Passive link check only. No STOP, motion or keepalive commands were sent; this does not certify the complete RL interface.')
    return 0 if result['capture_check_passed'] else 1


if __name__=='__main__':raise SystemExit(main())
