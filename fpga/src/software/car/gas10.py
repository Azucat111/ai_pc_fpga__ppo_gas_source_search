"""GAS10 wire schema. Acquisition validity is separate from model compatibility."""
import struct
import zlib
from .protocol import crc16,signed32

TEMPERATURES=(300,320,340,360,380,300,320,340,360,380)
DURATION_MS=100
PROFILE_ID=0x00010001
PROFILE_CRC=zlib.crc32(struct.pack('<10H H',*TEMPERATURES,DURATION_MS))&0xffffffff
GAS_SIGNATURE=0x47415331


def decode_gas10(packet):
    if len(packet)!=192 or packet[:4]!=b'\xa5\x5a\x01\x82':raise ValueError('Invalid GAS10 frame')
    if packet[188:190]!=b'\0\0' or crc16(packet[:-2])!=int.from_bytes(packet[-2:],'little'):raise ValueError('GAS10 CRC/reserved bytes')
    w=struct.unpack_from('<45I',packet,8)
    if w[7]&~1023 or w[8]!=1 or any(w[35:]):raise ValueError('Invalid GAS10 schema')
    profile_known=w[3]==PROFILE_ID and w[9]==DURATION_MS and w[34]==PROFILE_CRC
    values=list(w[10:20]);mask=w[7]
    flags=[(w[30]>>(3*i))&7 for i in range(10)]
    if any((mask>>i)&1 and (not values[i] or flags[i]!=7) for i in range(10)):raise ValueError('GAS10 mask inconsistent with point data')
    duration=(w[6]-w[5])&0xffffffff
    if any(((x-w[5])&0xffffffff)>duration for x in w[20:30]):raise ValueError('GAS10 point timestamps outside scan')
    if duration>30000:raise ValueError('GAS10 scan duration out of bounds')
    return dict(feature_type='GAS10',protocol_version=1,seq=int.from_bytes(packet[4:8],'little'),
                boot_id=w[0],board_timestamp_ms=w[1],scan_id=w[2],profile_id=w[3],sensor_id=w[4],
                scan_start_ms=w[5],scan_end_ms=w[6],scan_duration_ms=duration,gas_valid_mask=mask,
                acquisition_complete=mask==1023,profile_known=profile_known,
                acquisition_valid=bool(w[0] and mask==1023 and profile_known),
                gas10_ohm=values,point_timestamp_ms=list(w[20:30]),point_status=flags,
                heater_temperatures_c=list(TEMPERATURES) if profile_known else None,heater_duration_ms=w[9],
                temperature_c=signed32(w[31])/100,humidity_percent=w[32]/1000,pressure_pa=w[33],profile_crc32=w[34],
                model_profile_equivalence_verified=False,classification_valid=False,alcohol_prob=None)
