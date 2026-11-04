"""Fetch pinned, redistributable manufacturer sources; never execute them."""
from pathlib import Path
from urllib.request import urlopen
import hashlib
import json

ROOT = Path(__file__).resolve().parents[1]
BOSCH_REV = '80ea120a8b8ac987d7d79eb68a9ed796736be845'

def main():
    sources = {}
    for name in ('bme68x.c', 'bme68x.h', 'bme68x_defs.h', 'LICENSE'):
        sources['vendor/bme68x/' + name] = f'https://raw.githubusercontent.com/boschsensortec/BME68x_SensorAPI/{BOSCH_REV}/{name}'
    sources['vendor/pynq/pynqz2.tcl'] = 'https://raw.githubusercontent.com/Xilinx/PYNQ/v3.0.1/boards/Pynq-Z2/petalinux_bsp/hardware_project/pynqz2.tcl'
    sources['vendor/pynq/LICENSE'] = 'https://raw.githubusercontent.com/Xilinx/PYNQ/v3.0.1/LICENSE'
    manifest = {}
    for name, url in sources.items():
        path = ROOT / name
        path.parent.mkdir(parents=True, exist_ok=True)
        data = urlopen(url, timeout=30).read()
        path.write_bytes(data)
        manifest[name] = {'url': url, 'sha256': hashlib.sha256(data).hexdigest()}
        print(name, len(data))
    (ROOT/'vendor/manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')

if __name__ == '__main__':
    main()
