"""Board acceptance collector. Pops diagnostic FIFO only; no motion/LED commands.
Run only with newly implemented TGS1 overlay and no competing FIFO reader.
"""
import argparse,json,time,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'software'))
from car.tgs_features import TGSReader

def main():
    p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=60)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if not 1<=a.seconds<=3600:p.error('seconds must be 1..3600')
    from pynq import MMIO
    m=MMIO(0x43c00000,0x1000)
    if m.read(0)!=0x43415231:raise RuntimeError('CAR1 absent')
    reader=TGSReader(m);boot=m.read(0x10);before=m.read(0x508)
    a.output.mkdir(parents=True,exist_ok=False)
    start=time.perf_counter_ns();last=None;records=missing=invalid=0
    with (a.output/'records.jsonl').open('w',encoding='utf-8',buffering=65536) as f:
        while (time.perf_counter_ns()-start)/1e9<a.seconds:
            if m.read(0x10)!=boot:raise RuntimeError('Boot changed; do not merge sessions')
            for row in reader.drain():
                flags=row['flags'];updates=(flags>>2)&3;v=(flags>>6)&3
                if not updates or updates&~v:invalid+=1
                if last is not None:missing+=((row['record_id']-last-1)&0xffffffff)
                last=row['record_id'];records+=1
                row['boot_id']=boot;row['pc_elapsed_ns']=time.perf_counter_ns()-start
                f.write(json.dumps(row)+'\n')
            time.sleep(.01)
    summary=dict(boot_id=boot,elapsed_s=(time.perf_counter_ns()-start)/1e9,
        records=records,record_sequence_missing=missing,invalid_update_flags=invalid,
        overflow_delta=(m.read(0x508)-before)&0xffffffff,
        context=reader.context(),motion_commands_sent=0,
        note='Does not measure Wi-Fi, gas accuracy or physical motor response')
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2))
if __name__=='__main__':main()
