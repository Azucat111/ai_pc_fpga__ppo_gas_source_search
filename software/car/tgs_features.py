"""Optional single-reader PL FIFO exporter. Does not issue motion commands."""
import json
from pathlib import Path
import os
SIGNATURE=0x54475331

def signed(x):
    return x-(1<<32) if x & 0x80000000 else x

class TGSReader:
    def __init__(self,mmio):
        self.m=mmio
        if mmio.read(0x500)!=SIGNATURE: raise RuntimeError('TGS1 hardware absent')
    def drain(self,limit=16):
        result=[]
        for _ in range(limit):
            if not self.m.read(0x504): break
            w=[self.m.read(0x540+4*i) for i in range(16)]
            if w[0]!=SIGNATURE: raise RuntimeError('Invalid FIFO record')
            result.append(dict(record_id=w[1],flags=w[2],board_ms=w[3],
                raw=[signed(w[4]),signed(w[5])],sample_ms=w[6:8],
                filtered_q8=[signed(x) for x in w[8:10]],
                baseline_q8=[signed(x) for x in w[10:12]],
                delta_code=[signed(x) for x in w[12:14]],
                slope_code=[signed(x) for x in w[14:16]],overflow=self.m.read(0x508)))
            self.m.write(0x520,1)
        return result
    def context(self):
        # Retry if any channel updates during the multiword MMIO observation.
        for _ in range(8):
            before=[self.m.read(0x598),self.m.read(0x59c)]
            start=self.m.read(0x528)
            n=[self.m.read(0x52c),self.m.read(0x530)]
            peak=[self.m.read(0x534),self.m.read(0x538)]
            end=self.m.read(8)
            if before==[self.m.read(0x598),self.m.read(0x59c)]:
                return dict(window_start_ms=start,observed_end_ms=end,
                    samples=n,peak_abs_delta_code=peak,coherent=True)
        return dict(coherent=False)
    def start_window(self): self.m.write(0x524,1)

class TGSLog:
    def __init__(self,mmio,path,max_bytes=8*1024*1024):
        self.reader=TGSReader(mmio)
        self.path=Path(path);self.max_bytes=max_bytes
        self.size=self.path.stat().st_size if self.path.exists() else 0
        self.file=open(self.path,'a',encoding='utf-8',buffering=65536)
        self.boot=mmio.read(0x10)
    def poll(self):
        for row in self.reader.drain():
            row['boot_id']=self.boot
            line=json.dumps(row)+'\n'
            if self.size+len(line)>self.max_bytes:
                self.file.close()
                previous=Path(str(self.path)+'.1')
                if previous.exists():os.replace(previous,Path(str(self.path)+'.2'))
                if self.path.exists():os.replace(self.path,previous)
                self.file=open(self.path,'a',encoding='utf-8',buffering=65536);self.size=0
            self.file.write(line);self.size+=len(line)
    def close(self): self.file.close()
