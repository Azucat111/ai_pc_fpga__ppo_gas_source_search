import unittest
from car.tgs_features import TGSReader,SIGNATURE
class MMIO:
 def __init__(self):
  self.words=[SIGNATURE,5,12,100,0xfffffc18,1000,90,95]+[0]*8
  self.n=1
 def read(self,a):
  if a==0x500:return SIGNATURE
  if a==0x504:return self.n
  if 0x540<=a<0x580:return self.words[(a-0x540)//4]
  return 0
 def write(self,a,x):
  if a==0x520:self.n=0
class TGSTest(unittest.TestCase):
 def test_signed_decode_and_explicit_pop(self):
  m=MMIO();reader=TGSReader(m);rows=reader.drain()
  self.assertEqual(len(rows),1);self.assertEqual(rows[0]['raw'],[-1000,1000])
  self.assertEqual(rows[0]['sample_ms'],[90,95]);self.assertEqual(m.n,0)
 def test_rejects_absent_hardware(self):
  class Old:
   def read(self,a):return 0
  with self.assertRaises(RuntimeError):TGSReader(Old())
 def test_context_rejects_torn_observation(self):
  class Changing(MMIO):
   def read(self,a):
    if a==0x598:self.n+=1;return self.n
    return super().read(a)
  self.assertFalse(TGSReader(Changing()).context()['coherent'])

class RotationTest(unittest.TestCase):
 def test_live_log_has_bounded_backups(self):
  import tempfile,json
  from pathlib import Path
  from car.tgs_features import TGSLog
  with tempfile.TemporaryDirectory() as d:
   path=Path(d)/'live.jsonl';log=TGSLog(MMIO(),path,max_bytes=1024)
   for i in range(20):
    log.reader.m.n=1;log.poll()
   log.close()
   files=list(Path(d).iterdir())
   self.assertLessEqual(len(files),3)
   self.assertTrue(Path(str(path)+'.2').exists())
   for f in files:
    self.assertLessEqual(f.stat().st_size,1024)
    for row in f.read_text().splitlines():json.loads(row)
