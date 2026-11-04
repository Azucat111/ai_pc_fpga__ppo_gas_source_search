"""Reproducible dual-lane RTL replay; no hardware or network access."""
from pathlib import Path
import sys,random,os,subprocess,json,shutil,hashlib
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'software'))
from car.tgs_golden import Channel

def main():
    out=ROOT/'validation/golden';out.mkdir(parents=True,exist_ok=True)
    env=os.environ.copy()
    local=ROOT.parents[1]/'.tools/ucrt64/bin'
    env['PATH']=str(local)+os.pathsep+env.get('PATH','')
    rng=random.Random(20261101);rows=[];expect=[];a=Channel();b=Channel()
    for i in range(10000):
        reset=int(i in (0,1,2500,7500))
        # Directed steps cover both signs, release, extremes; random tail covers gaps/noise.
        if i<2000:
            x=1000 if i<100 or i>=1000 else 4000
            y=-x;v0=v1=1
        else:
            x=rng.randint(-32768,32767);y=rng.randint(-32768,32767)
            v0=int(rng.random()<.73);v1=int(rng.random()<.61)
        stamp=(0xffff0000+i*97)&0xffffffff
        rows.append(f'{reset} {v0} {x} {v1} {y} {stamp}\n')
        expect.append(a.step(reset,v0,x,stamp)+b.step(reset,v1,y,stamp))
    inp=out/'input.txt';actual=out/'rtl.txt';inp.write_text(''.join(rows))
    exe=out/'replay.vvp'
    commands=[[shutil.which('iverilog',path=env['PATH']),'-g2012','-s','tb_tgs_replay','-o',str(exe),str(ROOT/'rtl/tgs_channel.sv'),str(ROOT/'tests/tb_tgs_replay.sv')],
              [shutil.which('vvp',path=env['PATH']),str(exe),'+input='+str(inp),'+output='+str(actual)]]
    logs=[]
    for cmd in commands:
        p=subprocess.run(cmd,env=env,capture_output=True,text=True,timeout=120)
        logs.append(p.stdout+p.stderr)
        if p.returncode:raise RuntimeError(logs[-1])
    got=[[int(x) for x in row.split()] for row in actual.read_text().splitlines()]
    assert len(got)==len(expect),(len(got),len(expect))
    for i,(g,e) in enumerate(zip(got,expect)):
        if g!=e:raise AssertionError((i,g,e))
    result=dict(status='PASS',seed=20261101,cycles=len(rows),lanes=2,compared_fields=200000,
                max_integer_error=0,input_sha256=hashlib.sha256(inp.read_bytes()).hexdigest(),
                method='offline bit-exact replay; not physical throughput/latency measurement')
    (out/'result.json').write_text(json.dumps(result,indent=2))
    (out/'simulation.log').write_text(''.join(logs))
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
