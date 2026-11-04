"""Keep XSA outside the runtime BIT/HWH directory for PYNQ 3.1.1."""
from pathlib import Path
import hashlib,json

def main():
    root=Path(__file__).resolve().parents[1]
    src=root/'build/output/car.xsa';dest=root/'build/hardware_export/car.xsa'
    if not src.exists():
        if not dest.is_file():raise RuntimeError('No XSA export found')
        print('Runtime already prepared: BIT/HWH only');return
    dest.parent.mkdir(parents=True,exist_ok=True)
    h=hashlib.sha256(src.read_bytes()).hexdigest()
    if dest.exists():
        if hashlib.sha256(dest.read_bytes()).hexdigest()!=h:raise RuntimeError('Different XSA export already exists; preserve it separately first')
        src.unlink() # Identical exported copy verified before removing the duplicate.
    else:src.rename(dest)
    (root/'build/deployment_layout.json').write_text(json.dumps(dict(original='build/output/car.xsa',deployed='build/hardware_export/car.xsa',sha256=h),indent=2)+'\n')
    print('Preserved XSA separately; BIT/HWH unchanged')

if __name__=='__main__':main()
