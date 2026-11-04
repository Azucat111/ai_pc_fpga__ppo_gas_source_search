"""Local Windows passive capture with automatic Wi-Fi restoration; never sends car commands."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import socket
import sys
import uuid

from offline_ground_trial_windows import CarWiFi,IP,ROOT
from ubuntu_link_audit import collect
from car.config import load_config


def explain(capture):
    if capture.get('router_replies',{}).get('FCAR_FULL'):
        return '收到FCAR_FULL：登记已满；关闭旧接收程序，等待60秒后重测。'
    if capture.get('router_replies'):
        return '收到ESP登记/控制权回复，请查看router_replies。'
    if capture.get('unique_frames',0)==0:
        if capture.get('invalid_datagrams',0):return '有数据到达但不能解码为有效帧；查看原始包、CRC错误和协议版本。'
        return '没有收到有效状态帧；热点可连接不代表PYNQ程序及ESP-UART链路已工作。需结合路由源IP和error继续定位。'
    if not any(x.get('boot_id') for x in capture.get('boots',[])):
        return '收到状态帧但boot_id为0，板端尚未完成有效配置。'
    missing=[key for key,value in capture.get('fresh_valid_frame_count',{}).items() if value==0]
    if capture.get('gas10_valid_unique_scans',0)>0:
        return '已收到完整有效GAS10扫描；旧单点气体字段为null属于扫描模式预期。'+('本轮无有效值的其他传感器：'+','.join(missing)+'。' if missing else '')+'分类模型等价性仍需验证。'
    if missing:return '状态帧已收到，但以下传感器全程无有效值：'+','.join(missing)+'；这不同于无线完全收不到包。'
    if capture.get('bme_gas_usable_frames',0)==0:
        return '遥测与传感器状态可收，BME气体电阻尚无可用值；查看gas_valid/heater_stable。'
    return '本机已收到有效状态帧和传感器数据。跨电脑比较请先核对boot_id与板端采集状态，不能仅凭本机正常就认定Ubuntu故障。'


def run_check(directory,seconds=20,require_gas10=False):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=False)
    wifi=None;result=dict(restored=False,car_motion_commands_sent=0,car_stop_commands_sent=0)
    def log(message):
        print(message,flush=True)
        with (directory/'console.log').open('a',encoding='utf-8') as f:f.write(message+'\n')
    try:
        wifi=CarWiFi();log('自动连接FPGA-CAR热点，只收数据；请等待结束后自动恢复原Wi-Fi。')
        wifi.connect()
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as probe:
            probe.connect((IP,4242));result['route_source_ip']=probe.getsockname()[0]
        log('本机到小车的路由源IP：'+result['route_source_ip'])
        log('正在只读接收'+str(seconds)+'秒；不发运动、STOP或保活。')
        result['capture']=collect(IP,4242,load_config(ROOT/'config/car.json'),seconds,directory/'capture')
    except (OSError,ValueError,RuntimeError,TimeoutError,KeyboardInterrupt) as exc:
        result['error']=type(exc).__name__+': '+str(exc)
    finally:
        if wifi:
            log('正在恢复原Wi-Fi：'+wifi.restore_profile)
            try:result['restored']=wifi.restore()
            except Exception as exc:result['restore_error']=str(exc)
        c=result.get('capture',{})
        count=c.get('valid_frames_including_duplicates',0)+c.get('invalid_datagrams',0)+sum(c.get('router_replies',{}).values())+c.get('gas10_frames',0)
        result['gas10_required']=require_gas10
        result['gas10_check_passed']=c.get('gas10_valid_unique_scans',0)>=3
        result['ok']=bool(c.get('transport_check_passed') and result['restored'] and (not require_gas10 or result['gas10_check_passed']))
        lines=['板端启动编号：'+','.join(str(x['boot_id']) for x in c.get('boots',[])),
               '接收数据报：'+str(count),'有效去重状态帧：'+str(c.get('unique_frames',0)),
               '传感器有效帧：'+json.dumps(c.get('fresh_valid_frame_count',{}),ensure_ascii=False),
               '观察到的传感器更新数：'+json.dumps(c.get('observed_updates_by_reconstructed_commit_time',{}),ensure_ascii=False),
               'BME旧单点可用气体帧：'+str(c.get('bme_gas_usable_frames',0)),
               'GAS10帧：'+str(c.get('gas10_frames',0))+'；完整有效扫描：'+str(c.get('gas10_valid_unique_scans',0)),
               'GAS10专用检查：'+str(result['gas10_check_passed'])+'（至少3组；非分类精度验收）',
               'ESP回复：'+json.dumps(c.get('router_replies',{})),
               '错误：'+str(result.get('error') or c.get('error')),
               '原Wi-Fi恢复：'+str(result['restored']),
               '判断：'+(explain(c) if 'capture' in result else '未进入数据采集，请先看上面的错误。'),
               '结果目录：'+str(directory)]
        if not result['restored']:lines.append('请手动连接回原Wi-Fi后再返回对话。')
        (directory/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        (directory/'RESULT.txt').write_text('\n'.join(lines)+'\n',encoding='utf-8')
        log('\n'.join(lines))
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=int,default=20)
    p.add_argument('--require-gas10',action='store_true')
    p.add_argument('--dry-run',action='store_true')
    args=p.parse_args()
    if not 5<=args.seconds<=60:p.error('--seconds must be5..60')
    if args.dry_run:
        if args.require_gas10:print('GAS10 check requires at least3 complete valid unique scans; capture only, no motion commands.')
        print('DRY RUN: connectFPGA-CAR -> FCAR? registration and passive receive -> restore previous Wi-Fi. No motion/STOP/keepalive; no network change in dry run.')
        return 0
    if sys.platform!='win32':p.error('Use tools/ubuntu_link_audit.py directly on Ubuntu')
    directory=ROOT/'build/output'/('local_wifi_data_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
    return 0 if run_check(directory,args.seconds,args.require_gas10)['ok'] else 1


if __name__=='__main__':raise SystemExit(main())
