"""Temporary Windows wireless power setting; no sockets or car imports."""
import re,subprocess
WIRELESS='19cbb8fa-5279-450e-9fac-8a3d5fedd0c1'
SAVING='12bbebe6-58d6-4636-95bb-3217ef867c1a'
def power_command(*args):
 return subprocess.run(['powercfg',*args],capture_output=True,text=True,errors='replace',check=True).stdout
def performance_mode():
 active=power_command('/getactivescheme')
 scheme=re.search(r'[0-9a-fA-F]{8}(?:-[0-9a-fA-F]{4}){3}-[0-9a-fA-F]{12}',active).group()
 query=power_command('/query',scheme,WIRELESS,SAVING)
 indexes=re.findall(r'0x([0-9a-fA-F]{8})',query)
 if len(indexes)!=2:raise RuntimeError('Cannot read AC/DC power settings; nothing changed')
 ac,dc=[int(x,16) for x in indexes]
 power_command('/setdcvalueindex',scheme,WIRELESS,SAVING,'0')
 power_command('/setactive',scheme)
 return dict(scheme=scheme,ac=ac,dc=dc,override_dc=0)
def restore_power(saved):
 power_command('/setdcvalueindex',saved['scheme'],WIRELESS,SAVING,str(saved['dc']))
 power_command('/setactive',saved['scheme'])
