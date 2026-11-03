import json
import math
from pathlib import Path

TICK_S = 0.01

def load_config(path):
    c = json.loads(Path(path).read_text(encoding='utf-8'))
    # Older configs already encoded all sign corrections in encoder_*_invert.
    for key in ('encoder_m1_ab_swapped', 'encoder_m2_ab_swapped'):
        c.setdefault(key, False)
    for key in ('counts_per_revolution_left', 'counts_per_revolution_right'):
        if not isinstance(c[key], int) or isinstance(c[key], bool) or not 1 <= c[key] <= 1000000:
            raise ValueError(key + ' must be a positive integer <= 1000000')
    for key in ('wheel_diameter_left_m', 'wheel_diameter_right_m', 'track_width_m'):
        value = c[key]
        if value is not None and (isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not 0.005 <= value <= 5):
            raise ValueError(key + ' must be null or 0.005..5 metres')
    for key in ('encoder_m1_ab_swapped','encoder_m2_ab_swapped',
                'encoder_m1_invert','encoder_m2_invert','motor_m1_invert','motor_m2_invert',
                'wiring_confirmed','imu_yaw_invert','imu_gyro_z_invert','imu_mount_confirmed'):
        if not isinstance(c[key], bool): raise ValueError(key+' must be boolean')
    if c['m1_is_left'] is not None and not isinstance(c['m1_is_left'], bool):
        raise ValueError('m1_is_left must be true, false or null')
    bounds = {
        'wheel_kp': (0,1000), 'wheel_ki_per_tick':(0,100),
        'duty_limit_permille':(0,1000), 'deadzone_permille':(0,1000),
        'ramp_counts_per_tick':(0.01,100), 'max_wheel_rpm':(0.1,320),
        'heading_kp':(0,50), 'heading_kd':(0,50), 'max_turn_rate_rad_s':(0.01,3),
        'turn_tolerance_rad':(0.002,0.2), 'turn_rate_tolerance_rad_s':(0.001,1),
        'imu_timeout_ms':(50,1000), 'imu_yaw_offset_rad':(-math.pi,math.pi)}
    for key,(lo,hi) in bounds.items():
        v=c[key]
        if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not lo<=v<=hi:
            raise ValueError(f'{key} must be {lo}..{hi}')
    if c['deadzone_permille']>c['duty_limit_permille']:
        raise ValueError('deadzone exceeds duty limit')
    if c['imu_algorithm'] not in (6,9): raise ValueError('imu_algorithm must be 6 or 9')
    return c

def geometry_ready(c):
    return all(c[k] is not None for k in ('wheel_diameter_left_m','wheel_diameter_right_m','track_width_m'))

def rpm_target(rpm, c, side):
    if not math.isfinite(rpm) or abs(rpm)>c['max_wheel_rpm']:
        raise ValueError('Requested rpm exceeds configured maximum')
    value=round(rpm*c['counts_per_revolution_'+side]*TICK_S/60*65536)
    if abs(value)>200*65536: raise ValueError('Counts per tick exceed implementation limit')
    return value

def velocity_targets(v, omega, c):
    if not geometry_ready(c): raise ValueError('Fill wheel diameters and track_width_m first')
    if not math.isfinite(v) or not math.isfinite(omega): raise ValueError('Non-finite velocity')
    b=c['track_width_m']
    left=(v-omega*b/2)*60/(math.pi*c['wheel_diameter_left_m'])
    right=(v+omega*b/2)*60/(math.pi*c['wheel_diameter_right_m'])
    return rpm_target(left,c,'left'),rpm_target(right,c,'right')

def registers(c):
    # Swapping A/B reverses the quadrature sign. Compose that known wiring
    # correction with the independently calibrated forward sign, once only.
    enc1 = c.get('encoder_m1_ab_swapped', False) ^ c['encoder_m1_invert']
    enc2 = c.get('encoder_m2_ab_swapped', False) ^ c['encoder_m2_invert']
    flags=(int(c['m1_is_left'] is False) | (int(enc1)<<1) |
           (int(enc2)<<2) | (int(c['motor_m1_invert'])<<3) |
           (int(c['motor_m2_invert'])<<4))
    maximum=max(rpm_target(c['max_wheel_rpm'],c,s) for s in ('left','right'))
    r={0x18:flags,0x1c:round(c['wheel_kp']*256),0x20:round(c['wheel_ki_per_tick']*256),
       0x24:int(c['duty_limit_permille']) | (int(c['deadzone_permille'])<<16),
       0x28:round(c['ramp_counts_per_tick']*65536),0x2c:maximum,
       0x30:round(c['heading_kp']*256),0x34:round(c['heading_kd']*256),
       0x38:round(c['max_turn_rate_rad_s']*1000),
       0x3c:round(c['turn_tolerance_rad']*1000)|(round(c['turn_rate_tolerance_rad_s']*1000)<<16),
       0x40:0,0x44:0,0x48:0,0x4c:0,0x50:0,
       0x54:int(c['imu_yaw_invert'])|(int(c['imu_gyro_z_invert'])<<1),0x58:round(c['imu_yaw_offset_rad']*1000)&0xffffffff,
       0x5c:int(c['imu_timeout_ms']),0x60:0}
    if geometry_ready(c):
        for side,scale_addr,factor_addr in [('left',0x40,0x4c),('right',0x44,0x50)]:
            diameter=c['wheel_diameter_'+side+'_m'];cpr=c['counts_per_revolution_'+side]
            r[scale_addr]=round(math.pi*diameter/cpr*1e6*65536)
            r[factor_addr]=round(c['track_width_m']/2/(math.pi*diameter)*cpr*TICK_S*65536)
        r[0x48]=round(1000/(c['track_width_m']*1e6)*2**24)
        r[0x60]=1|(int(c['imu_mount_confirmed'])<<1)
    if any(not 0<=value<=0xffffffff for value in r.values()):
        raise ValueError('Configuration does not fit FPGA registers')
    return r
