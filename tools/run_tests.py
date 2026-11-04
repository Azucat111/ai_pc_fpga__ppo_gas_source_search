"""Portable checks: Python standard library + Icarus Verilog (no Vivado needed)."""
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'software'))
from car.protocol import command
from car.source_event import event_packet

def main():
    os.chdir(ROOT)
    out=ROOT/'build/tests';out.mkdir(parents=True,exist_ok=True)
    local=ROOT/'.tools/ucrt64/bin'
    env=os.environ.copy()
    if local.exists():env['PATH']=str(local)+os.pathsep+env.get('PATH','')
    env['PYTHONPATH']=str(ROOT/'software');env['PYTHONIOENCODING']='utf-8';env['PYTHONUTF8']='1'
    iverilog=shutil.which('iverilog',path=env['PATH']);vvp=shutil.which('vvp',path=env['PATH'])
    if not iverilog or not vvp:raise SystemExit('Install Icarus Verilog; on Windows run tools/bootstrap_sim_windows.py')
    (out/'command.hex').write_text('\n'.join(f'{x:02x}' for x in command(1,0x12345678,0,1,-131072,65536))+'\n')
    (out/'integration_command.hex').write_text('\n'.join(f'{x:02x}' for x in command(1,0x12345678,0,1,131072,131072,100))+'\n')
    (out/'pwm_diagnostic_command.hex').write_text('\n'.join(f'{x:02x}' for x in command(1,0x12345678,0,1,0,144179,100))+'\n')
    (out/'pwm_diagnostic_reverse_command.hex').write_text('\n'.join(f'{x:02x}' for x in command(2,0x12345678,0,1,0,-144179,100))+'\n')
    (out/'pwm_diagnostic_both_command.hex').write_text('\n'.join(f'{x:02x}' for x in command(3,0x12345678,0,1,144179,144179,100))+'\n')
    (out/'pwm_diagnostic_left_command.hex').write_text('\n'.join(f'{x:02x}' for x in command(4,0x12345678,0,1,144179,0,100))+'\n')
    (out/'source_event.hex').write_text('\n'.join(f'{x:02x}' for x in event_packet(900,0x12345678,10,1))+'\n')
    results=[]
    def run(args,timeout=120):
        r=subprocess.run(args,env=env,capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=timeout)
        if r.stdout.strip():print(r.stdout.strip(),flush=True)
        if r.returncode or r.stderr.strip():print(r.stderr.strip(),flush=True)
        results.append('$ '+' '.join(map(str,args))+'\n'+r.stdout+r.stderr)
        (out/'results.txt').write_text('\n'.join(results),encoding='utf-8')
        if r.returncode:raise SystemExit(r.returncode)
    rtl=[str(p) for p in sorted((ROOT/'rtl').glob('*.sv'))]
    run([iverilog,'-g2012','-s','car_axi','-o',str(out/'car.vvp')]+rtl)
    for bench in sorted((ROOT/'tests').glob('tb_*.sv')):
        target=out/(bench.stem+'.vvp')
        run([iverilog,'-g2012','-s',bench.stem,'-o',str(target)]+rtl+[str(bench),str(ROOT/'tests/i2c_slave_model.sv')])
        run([vvp,str(target)])
    run([sys.executable,'-m','unittest','discover','-s','tests','-p','test_*.py','-v'])
    print('All available checks passed. Vivado timing and physical hardware remain separate checks.')

if __name__=='__main__':main()
