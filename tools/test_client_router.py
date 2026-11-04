"""Compile and execute the actual ESP arbitration header on a native C++ compiler."""
from pathlib import Path
import os
import shlex
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[1]

def main():
    compiler=os.environ.get('CXX')
    zig=ROOT/'.tools/native-compiler/ziglang/zig.exe'
    command=shlex.split(compiler) if compiler else ([str(zig),'c++'] if zig.exists() else [shutil.which('c++') or 'g++'])
    output=ROOT/'build/tests';output.mkdir(parents=True,exist_ok=True)
    binary=output/('client_router_test.exe' if os.name=='nt' else 'client_router_test')
    args=command+['-std=c++11','-Wall','-Wextra','-Werror','-I'+str(ROOT/'firmware/esp32c3/include'),
                  str(ROOT/'tests/client_router_test.cpp'),'-o',str(binary)]
    records=[]
    for argv in (args,[str(binary)]):
        result=subprocess.run(argv,cwd=ROOT,capture_output=True,text=True,timeout=180)
        records.append(result.stdout+result.stderr)
        (output/'client_router.txt').write_text('\n'.join(records),encoding='utf-8')
        print(result.stdout+result.stderr,end='')
        if result.returncode:raise SystemExit(result.returncode)

if __name__=='__main__':main()
