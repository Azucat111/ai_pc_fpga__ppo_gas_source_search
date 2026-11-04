"""Offline RTL and Python regression. Does not connect to the car."""
import hashlib,json,os,subprocess,sys,unittest
from pathlib import Path

def main():
    root=Path(__file__).resolve().parents[1];original=root.parents[1]
    os.chdir(root);sys.path[:0]=[str(root/'software'),str(root/'tests')]
    out=root/'build/tests';out.mkdir(parents=True,exist_ok=True)
    from car.protocol import command
    from car.source_event import event_packet
    for name,p in [('command',command(1,0x12345678,0,1,-131072,65536)),
                   ('integration_command',command(1,0x12345678,0,1,131072,131072,100)),
                   ('source_event',event_packet(900,0x12345678,10,1)),
                   ('pwm_diagnostic_command',command(1,0x12345678,0,1,0,144179,100)),
                   ('pwm_diagnostic_reverse_command',command(2,0x12345678,0,1,0,-144179,100)),
                   ('pwm_diagnostic_both_command',command(3,0x12345678,0,1,144179,144179,100)),
                   ('pwm_diagnostic_left_command',command(4,0x12345678,0,1,144179,0,100))]:
        (out/(name+'.hex')).write_text('\n'.join(f'{x:02x}' for x in p)+'\n')
    env=os.environ.copy();env['PYTHONUTF8']='1'
    env['PATH']=str(original/'.tools/ucrt64/bin')+os.pathsep+env['PATH']
    logs=[]
    for bench in sorted((root/'tests').glob('tb_*.sv')):
        binary=out/(bench.stem+'.vvp')
        commands=[['iverilog','-g2012','-s',bench.stem,'-o',str(binary)]+
                  [str(p) for p in sorted((root/'rtl').glob('*.sv'))]+[str(bench),str(root/'tests/i2c_slave_model.sv')],
                  ['vvp',str(binary)]]
        for argv in commands:
            argv[0]=str(original/'.tools/ucrt64/bin'/(argv[0]+'.exe'))
            r=subprocess.run(argv,env=env,capture_output=True,text=True,timeout=180)
            logs.append(r.stdout+r.stderr)
            (out/'led_rtl_regression.txt').write_text('\n'.join(logs),encoding='utf-8')
            if r.returncode:print(r.stdout+r.stderr);raise SystemExit(r.returncode)
        print(bench.stem+' PASS',flush=True)
    baseline=json.loads((root/'baseline_hashes.json').read_text())
    for name,h in baseline.items():assert hashlib.sha256((root/name).read_bytes()).hexdigest()==h,name
    suite=unittest.TestSuite();loader=unittest.TestLoader()
    # No sockets to external hosts or actual loopback tests in this restricted environment.
    for name in ('test_source_event','test_software','test_host_keepalive','test_multi_client','test_teleop'):
        suite.addTests(loader.loadTestsFromName(name))
    with (out/'led_python_regression.txt').open('w',encoding='utf-8') as log:
        result=unittest.TextTestRunner(stream=log,verbosity=2).run(suite)
    print('Python:',result.testsRun,'tests; success:',result.wasSuccessful())
    if not result.wasSuccessful():print((out/'led_python_regression.txt').read_text());raise SystemExit(1)
    print('Protected motor/encoder/config/parser hashes unchanged; no physical tests performed.')

if __name__=='__main__':main()
