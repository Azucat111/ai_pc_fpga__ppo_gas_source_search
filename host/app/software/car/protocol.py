import binascii
import struct
import math

SYNC=b'\xa5\x5a'
VERSION=1
COMMAND_BYTES=32
TELEMETRY_BYTES=192
MODES={0:'stop',1:'wheel_velocity',2:'straight',3:'turn_relative'}

def crc16(data):
    return binascii.crc_hqx(data,0xffff)

def command(seq,boot_id,issued_ms,mode,p0=0,p1=0,ttl_ms=500):
    if mode not in range(5): raise ValueError('Unknown mode')
    if not 100<=ttl_ms<=500: raise ValueError('TTL must be 100..500 ms')
    body=struct.pack('<2sBBIIIBBHiiH',SYNC,1,1,seq&0xffffffff,boot_id&0xffffffff,
                     issued_ms&0xffffffff,mode,0,ttl_ms,p0,p1,0)
    return body+struct.pack('<H',crc16(body))

def signed32(x):
    x &= 0xffffffff
    return x-(1<<32) if x&(1<<31) else x
def signed64(lo,hi):
    x=lo|(hi<<32)
    return x-(1<<64) if x&(1<<63) else x

def decode(packet,c):
    if len(packet)!=192 or packet[:4]!=SYNC+b'\x01\x81': raise ValueError('Invalid telemetry header')
    if crc16(packet[:-2])!=struct.unpack_from('<H',packet,190)[0]: raise ValueError('CRC mismatch')
    if packet[188:190]!=b'\0\0': raise ValueError('Reserved bytes nonzero')
    w=struct.unpack_from('<45I',packet,8)
    return decode_snapshot(struct.unpack_from('<I',packet,4)[0],w,c)

def decode_snapshot(seq,w,c):
    """Decode the same coherent 45-word snapshot from UART or local AXI."""
    if len(w)!=45: raise ValueError('A snapshot requires 45 words')
    flags=w[7];geo=bool(flags&16)
    si=lambda i:signed32(w[i])
    count_l=signed64(w[8],w[9]);count_r=signed64(w[10],w[11])
    speeds=[];rpms=[]
    for side,idx in [('left',14),('right',15)]:
        cps=si(idx)/65536*100
        rpms.append(cps*60/c['counts_per_revolution_'+side])
        diameter=c.get('wheel_diameter_'+side+'_m')
        speeds.append(cps*math.pi*diameter/c['counts_per_revolution_'+side] if geo and diameter is not None else None)
    bvalid=bool(flags&32);gasvalid=bvalid and bool(w[28]&2);heatstable=bvalid and bool(w[28]&4)
    result={
        'protocol_version':1,'seq':seq,
        'boot_id':w[0],'timestamp_ms':w[1],'last_command_seq':w[2],'action_id':w[3],
        'last_action':MODES.get(w[4],'unknown'),'action_age_ms':w[5],
        'stop_reason':{0:'command',1:'disabled',2:'command_timeout',3:'imu_unavailable',4:'turn_complete'}.get(w[6],'unknown'),
        'enabled':bool(flags&1),'geometry_valid':geo,'imu_frame_confirmed':bool(flags&64),
        'source_led_supported':bool(flags&1024),
        'bme_gas10_mode':bool(flags&512),
        'turn_gyro_heading':bool(flags&256),
        'straight_gyro_heading':bool(flags&128),
        'encoder_left_count':count_l,'encoder_right_count':count_r,
        'encoder_left_delta':si(12),'encoder_right_delta':si(13),
        'wheel_speed_left':speeds[0],'wheel_speed_right':speeds[1],
        'wheel_rpm_left':rpms[0],'wheel_rpm_right':rpms[1],
        'linear_velocity':si(33)/1e6 if geo else None,
        'angular_velocity':si(34)/1000 if geo else None,
        'imu_yaw':si(16)/1000 if flags&2 else None,
        'imu_gyro_z':si(17)/1000 if flags&2 else None,'imu_age_ms':w[18],
        'distance_m':(signed64(w[29],w[30])+signed64(w[31],w[32]))/(2*65536*1e6) if geo else None,
        'travelled_distance_m':signed64(w[35],w[36])/(65536*1e6) if geo else None,
        'snapshot_dt_ms':w[37],'command_status':w[38],'last_checked_command_seq':w[42],
        'errors':{'imu':w[39],'adc':w[40],'uart':w[41],'encoder_m1':w[43],'encoder_m2':w[44]},
        'tgs2602':[],
        'bme688':{'temperature_c':si(23)/100 if bvalid else None,
                  'humidity_percent':w[24]/1000 if bvalid else None,
                  'pressure_pa':w[25] if bvalid else None,
                  'gas_resistance_ohm':w[26] if gasvalid and heatstable else None,
                  'valid':bvalid,'gas_valid':gasvalid,'heater_stable':heatstable,'age_ms':w[27]}}
    for i in range(2):
        valid=bool(flags&(4<<i));raw=si(19+i)
        result['tgs2602'].append({'channel':i,'raw':raw if valid else None,
          'voltage_v':raw*6.144/32768 if valid else None,'valid':valid,'age_ms':w[21+i],
          'saturated':valid and abs(raw)>=32767})
    return result
