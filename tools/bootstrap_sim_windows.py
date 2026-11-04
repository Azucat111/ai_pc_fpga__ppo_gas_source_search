"""Install an isolated MSYS2 Icarus runtime under .tools, without PATH changes."""
from pathlib import Path
from urllib.request import urlopen, urlretrieve
from urllib.parse import unquote, quote
import re
import subprocess

ROOT=Path(__file__).resolve().parents[1]/'.tools'
BASE='https://repo.msys2.org/mingw/ucrt64/'

def main():
    ROOT.mkdir(exist_ok=True)
    index=urlopen(BASE, timeout=30).read().decode()
    names=[unquote(n) for n in re.findall(r'href="([^"/]+\.pkg\.tar\.zst)"',index)]
    visited=set()
    def install(package):
        if package in visited:return
        visited.add(package)
        candidates=[n for n in names if n.startswith(package+'-') and re.match(r'\d',n[len(package)+1:])]
        if not candidates: raise RuntimeError('Package not found: '+package)
        # Repository directory is sorted; natural numeric order avoids 9 > 10.
        def key(n):return [int(x) if x.isdigit() else x for x in re.split(r'(\d+)',n)]
        name=max(candidates,key=key)
        archive=ROOT/name
        if not archive.exists():urlretrieve(BASE+quote(name),archive)
        info=subprocess.check_output(['tar','-xOf',str(archive),'.PKGINFO'],text=True)
        subprocess.run(['tar','-xf',str(archive),'-C',str(ROOT)],check=True)
        print(name,flush=True)
        for dep in re.findall(r'^depend = (.+)$',info,re.M):
            install(re.split('[<>=]',dep)[0])
    for p in ['iverilog','libgcc','libstdc++']:
        install('mingw-w64-ucrt-x86_64-'+p)

if __name__=='__main__':main()
