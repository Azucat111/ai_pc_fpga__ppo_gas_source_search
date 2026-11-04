"""Operator-run Windows trial: local confirmation, bounded motion, STOP, Wi-Fi restore."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import socket
import subprocess
import sys
import time
import uuid
from xml.sax.saxutils import escape

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'software'))
from car.config import load_config
from car.host import Client
from trial_transport import UDPTransport
from wifi_smoke_test_windows import netsh

IP='192.168.4.1'
SETTINGS=ROOT/'config/windows_wifi_restore.json'


def wifi_state():
    text=netsh('show','interfaces')
    def field(pattern):
        values=re.findall(pattern,text,re.M)
        if len(values)>1:raise RuntimeError('仅支持一块Wi-Fi网卡，请先明确使用的网卡。')
        return values[0].strip() if values else None
    return dict(profile=field(r'^\s*(?:Profile|配置文件)\s*:\s*(.+?)\s*$'),
                ssid=field(r'^\s*SSID\s*:\s*(.+?)\s*$'),
                interface=field(r'^\s*(?:Name|名称)\s*:\s*(.+?)\s*$'))


def credentials():
    text=(ROOT/'firmware/esp32c3/include/wifi_config.h').read_text(encoding='utf-8')
    values=[]
    for key in ('SSID','PASSWORD'):
        m=re.search(r'^#define\s+CAR_WIFI_'+key+r'\s+"([^"\n]+)"',text,re.M)
        if not m:raise RuntimeError('找不到项目热点配置。')
        values.append(m.group(1))
    return values


class CarWiFi:
    def __init__(self,settings=SETTINGS,build_dir=ROOT/'build'):
        self.settings=Path(settings);self.build_dir=Path(build_dir)
        self.ssid,self.password=credentials();self.original=wifi_state()
        cached=json.loads(self.settings.read_text(encoding='utf-8')) if self.settings.is_file() else {}
        self.interface=self.original['interface'] or cached.get('interface','WLAN')
        if self.original['ssid']!=self.ssid and self.original['profile']:
            self.restore_profile=self.original['profile']
            self.settings.parent.mkdir(parents=True,exist_ok=True)
            self.settings.write_text(json.dumps(dict(profile=self.restore_profile,interface=self.interface),ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        else:
            self.restore_profile=cached.get('profile')
        if not self.restore_profile:
            raise RuntimeError('无法确定原Wi-Fi。请先连回正常网络再启动脚本。')
        self.profile='FPGA-CAR-local-'+uuid.uuid4().hex[:10]
        self.xml=self.build_dir/(self.profile+'.xml')
        self.added=False;self.switched=False;self.on_car=self.original['ssid']==self.ssid
        self.restored=False;self.cleanup_error=None

    def connect(self):
        if self.on_car:return
        self.build_dir.mkdir(parents=True,exist_ok=True)
        self.xml.write_text('''<?xml version="1.0"?>
<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
<name>'''+escape(self.profile)+'''</name><SSIDConfig><SSID><name>'''+escape(self.ssid)+'''</name></SSID></SSIDConfig>
<connectionType>ESS</connectionType><connectionMode>manual</connectionMode>
<MSM><security><authEncryption><authentication>WPA2PSK</authentication><encryption>AES</encryption><useOneX>false</useOneX></authEncryption>
<sharedKey><keyType>passPhrase</keyType><protected>false</protected><keyMaterial>'''+escape(self.password)+'''</keyMaterial></sharedKey>
</security></MSM></WLANProfile>''',encoding='utf-8')
        netsh('add','profile','filename='+str(self.xml),'user=current');self.added=True
        self.switched=True
        netsh('connect','name='+self.profile,'interface='+self.interface)
        deadline=time.monotonic()+25
        while time.monotonic()<deadline:
            try:
                if wifi_state()['ssid']==self.ssid:
                    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
                        s.connect((IP,4242))
                        if s.getsockname()[0].startswith('192.168.4.'):
                            self.on_car=True;return
            except (OSError,RuntimeError):pass
            time.sleep(.5)
        raise TimeoutError('连接小车热点或获取IP超时，请检查PYNQ/ESP32是否已启动。')

    def restore(self):
        if self.switched or self.on_car:
            for _ in range(3):
                try:
                    netsh('connect','name='+self.restore_profile,'interface='+self.interface)
                    for _ in range(20):
                        time.sleep(.5)
                        if wifi_state()['profile']==self.restore_profile:
                            self.restored=True;break
                except (RuntimeError,subprocess.TimeoutExpired):pass
                if self.restored:break
        else:self.restored=True
        if self.added:
            try:netsh('delete','profile','name='+self.profile,'interface='+self.interface)
            except Exception as e:self.cleanup_error=str(e)
        self.xml.unlink(missing_ok=True)
        return self.restored


def read_and_stop(config):
    client=Client(IP,config,sock=UDPTransport(IP))
    try:
        client.receive(timeout=5)
        state=client.send(0)
        if state['last_action']!='stop':raise RuntimeError('没有确认STOP，取消测试。')
        return state
    finally:client.close()


def validate_preflight(state):
    if not state['boot_id'] or not state['enabled']:
        raise RuntimeError('小车未进入无线待命；请先完成PYNQ启动。')
    if not state['geometry_valid'] or not state['imu_frame_confirmed']:
        raise RuntimeError('车辆参数或IMU安装配置未就绪。')
    if state['imu_yaw'] is None or state['imu_age_ms']>150:
        raise RuntimeError('IMU数据无效或超时，取消测试。')
    if any(abs(state['wheel_rpm_'+side])>1 for side in ('left','right')):
        raise RuntimeError('车轮尚未停稳，取消测试。')
    if not all(v['valid'] for v in state['tgs2602']):
        raise RuntimeError('电机侧传感器电源未就绪，请接通扩展板12V供电；本次不发运动指令。')


def show_recent_completed_turn(state,session_dir,degrees,log):
    # Display a receipt only; this never replays, suppresses, or sends commands.
    candidates=list(Path(session_dir).parent.glob('user_ground_*/trial/summary.json'))
    for path in sorted(candidates,key=lambda p:p.stat().st_mtime,reverse=True)[:30]:
        try:
            age=time.time()-path.stat().st_mtime
            if not 0<=age<=300:continue
            previous=json.loads(path.read_text(encoding='utf-8'))
            if (previous.get('boot_id')!=state['boot_id'] or previous.get('target_degrees')!=degrees
                or not previous.get('completed_by_fpga') or not previous.get('stop_acknowledged')):continue
            angle=float(previous['gyro_turn_degrees'])
            when=datetime.fromtimestamp(path.stat().st_mtime).strftime('%H:%M:%S')
        except (OSError,ValueError,KeyError,TypeError):continue
        log('上一轮同方向动作已完成（'+when+'）：陀螺记录'+format(angle,'.2f')+'度，STOP已确认。')
        log('本次输入GO会从当前位置再转'+str(abs(degrees))+'度，请重新标记起始朝向。')
        break


def run_ground_trial(state,session_dir,log,input_fn=input,turn_degrees=None,straight_seconds=3):
    validate_preflight(state)
    log('已连接小车，STOP已确认。启动编号：'+str(state['boot_id']))
    if turn_degrees is None:
        log('测试：20 rpm，最多'+str(straight_seconds)+'秒；轮式里程上限0.5米、偏航保护15度。')
        log('确认车已落地，前方至少1米、左右各半米无障碍，IMU固定、没有电脑连线。')
    else:
        direction='左（俯视逆时针）' if turn_degrees>0 else '右（俯视顺时针）'
        log('测试：原地向'+direction+'转'+str(abs(turn_degrees))+'度，最长10秒，完成即STOP。')
        log('确认车已落地，四周至少半米无障碍，IMU固定、没有电脑连线。')
        show_recent_completed_turn(state,session_dir,turn_degrees,log)
    log('输入 GO 后回车才会运动；其他输入取消。运行时 Ctrl+C 请求STOP。')
    if input_fn('准备好观察小车后输入 GO：').strip().upper()!='GO':
        return dict(cancelled=True,child_returncode=None)
    operator_countdown(log)
    log('开始：先静止检查约1秒，再'+('低速前进' if turn_degrees is None else '原地转向')+'，随后自动STOP。请观察小车。')
    return execute_motion_step(state,session_dir,log,turn_degrees,straight_seconds)


def operator_countdown(log):
    for n in (3,2,1):
        log(str(n)+'…')
        try:
            import winsound
            winsound.Beep(800,100)
        except (ImportError,RuntimeError):pass
        time.sleep(1)


def execute_motion_step(state,session_dir,log,turn_degrees=None,straight_seconds=3,stop_file=None):
    output=Path(session_dir)/'trial'
    command=[sys.executable,'-X','utf8',str(ROOT/'tools/check_ground_straight.py'),
             '--ip',IP,'--boot-id',str(state['boot_id']),'--ground-clear',
             '--rpm','20','--seconds',str(straight_seconds),'--output',str(output),
             '--stop-file',str(stop_file or Path(session_dir)/'STOP.request')]
    if turn_degrees is not None:
        command=[sys.executable,'-X','utf8',str(ROOT/'tools/check_ground_turn.py'),
                 '--ip',IP,'--boot-id',str(state['boot_id']),'--ground-clear',
                 '--degrees',str(turn_degrees),'--seconds','10','--output',str(output),
                 '--stop-file',str(stop_file or Path(session_dir)/'STOP.request')]
    child=subprocess.run(command,cwd=ROOT,timeout=25)
    if turn_degrees is not None:
        try:
            done=json.loads((output/'summary.json').read_text(encoding='utf-8'))
            if done.get('completed_by_fpga') and done.get('stop_acknowledged'):
                log('本轮转角动作已完成：陀螺记录'+format(float(done['gyro_turn_degrees']),'.2f')+'度，STOP已确认。重新运行会再转一次。')
        except (OSError,ValueError,KeyError,TypeError):pass
    return dict(cancelled=False,child_returncode=child.returncode,command=command)


def run_l_route(state,session_dir,config,log,input_fn=input):
    validate_preflight(state)
    if not state.get('straight_gyro_heading') or not state.get('turn_gyro_heading'):
        raise RuntimeError('L形组合需要已验证的v018直行和转弯控制。')
    log('组合测试：直行20rpm/6秒 → 停车 → 左转90度 → 停车 → 直行20rpm/6秒 → 停车。')
    log('每段直行预计30～40厘米；请留出约1米×1米平整空地，并标记起点和朝向。')
    log('一次GO授权以上三段；任何一段检查失败就结束，不继续后续动作。')
    if input_fn('准备好观察完整L形动作后输入 GO：').strip().upper()!='GO':
        log('已取消L形组合，未发送运动指令。')
        return dict(cancelled=True,child_returncode=None)
    operator_countdown(log)
    directory=Path(session_dir);stop_file=directory/'STOP.request'
    summary=dict(ok=False,mode='route_l',boot_id=state['boot_id'],steps=[],
                 validation_scope='bounded_sequence_encoder_and_imu_checks_only',physical_movement_confirmed=None)
    try:
        for index,(label,degrees) in enumerate((('前进6秒',None),('左转90度',90),('前进6秒',None)),1):
            if stop_file.exists():raise KeyboardInterrupt('L形STOP文件已请求')
            fresh=read_and_stop(config)
            if fresh['boot_id']!=state['boot_id']:raise RuntimeError('组合中控制器已重启，后续动作取消。')
            validate_preflight(fresh)
            log('第'+str(index)+'/3段开始：'+label)
            phase_dir=directory/('step_'+str(index))
            child=execute_motion_step(fresh,phase_dir,log,degrees,6,stop_file=stop_file)
            part=json.loads((phase_dir/'trial/summary.json').read_text(encoding='utf-8'))
            summary['steps'].append(dict(index=index,label=label,child_returncode=child['child_returncode'],result=part))
            if (child['child_returncode']!=0 or not part.get('ok') or not part.get('stop_acknowledged')
                or part.get('boot_id')!=state['boot_id']):
                raise RuntimeError('第'+str(index)+'段未通过并停车，后续动作取消。')
            log('第'+str(index)+'/3段完成，STOP已确认。')
        summary['ok']=True
        log('本轮L形三段已全部执行并停车。')
        return dict(cancelled=False,child_returncode=0)
    except (OSError,ValueError,RuntimeError,TimeoutError,subprocess.TimeoutExpired,KeyboardInterrupt) as exc:
        summary['error']=type(exc).__name__+': '+str(exc)
        raise
    finally:
        out=directory/'trial';out.mkdir(parents=True,exist_ok=True)
        (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


def execute_session(session_dir,stop_only=False,input_fn=input,turn_degrees=None,observe_still=False,straight_seconds=3,route_l=False):
    session_dir=Path(session_dir);session_dir.mkdir(parents=True,exist_ok=False)
    config=load_config(ROOT/'config/car.json')
    result=dict(stop_only=stop_only,turn_degrees=turn_degrees,observe_still=observe_still,straight_seconds=straight_seconds,route_l=route_l,ok=False,restored=False)
    wifi=None
    def log(message):
        print(message,flush=True)
        with (session_dir/'console.log').open('a',encoding='utf-8') as f:f.write(message+'\n')
    try:
        wifi=CarWiFi();log('正在连接小车热点；期间互联网可能暂时断开。')
        wifi.connect()
        state=read_and_stop(config);result['preflight']=state
        if stop_only:
            result.update(ok=True,stop_acknowledged=True);log('STOP已获小车确认。')
        elif route_l:
            result.update(run_l_route(state,session_dir,config,log,input_fn))
            result['ok']=result.get('child_returncode')==0
        elif observe_still:
            log('静止读取12秒：只发STOP，不发运动、不改标定。保持车身和IMU不动，供电接法不变。')
            if input_fn('准备好保持静止后输入 READ：').strip().upper()!='READ':
                result['cancelled']=True;log('已取消读取。')
            else:
                child=subprocess.run([sys.executable,'-X','utf8',str(ROOT/'tools/check_imu_still.py'),
                                      '--ip',IP,'--output',str(session_dir/'trial')],cwd=ROOT,timeout=25)
                result.update(ok=child.returncode==0,child_returncode=child.returncode)
        else:
            trial=run_ground_trial(state,session_dir,log,input_fn,turn_degrees,straight_seconds)
            result.update(trial)
            result['ok']=trial.get('child_returncode')==0
            if trial.get('cancelled'):log('已取消，未发送运动指令。')
    except (OSError,ValueError,RuntimeError,TimeoutError,subprocess.TimeoutExpired,KeyboardInterrupt,EOFError) as e:
        result['error']=type(e).__name__+': '+str(e);log('已中止：'+str(e))
    finally:
        if wifi and wifi.on_car:
            try:
                stopped=read_and_stop(config);result['final_stop_acknowledged']=stopped['last_action']=='stop'
                log('结束：STOP已确认。')
            except (OSError,RuntimeError,TimeoutError) as e:
                result['stop_error']=str(e);log('未收到STOP确认；请立即断开小车总电源。FPGA另有500毫秒保活超时停车。')
        if wifi:
            log('正在恢复原Wi-Fi：'+wifi.restore_profile)
            try:result['restored']=wifi.restore()
            except Exception as e:result['restore_error']=str(e)
            if wifi.cleanup_error:result['profile_cleanup_error']=wifi.cleanup_error
        result['ok']=result['ok'] and result.get('final_stop_acknowledged',False) and result['restored']
        result['physical_movement_confirmed']=None
        (session_dir/'session_summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        lines=['STOP确认：'+str(result.get('final_stop_acknowledged',False)),
               '原Wi-Fi恢复：'+str(result['restored'])]
        trial_summary=session_dir/'trial/summary.json'
        if trial_summary.is_file():
            s=json.loads(trial_summary.read_text(encoding='utf-8'))
            lines.append('软件数据检查：'+str(s.get('ok',False)))
            if s.get('validation_scope')=='stationary_observation_only':
                lines.extend(['静止航向变化：'+str(s.get('yaw_drift_deg'))+' deg',
                              '静止航向范围：'+str(s.get('yaw_range_deg'))+' deg',
                              '读取完整性检查通过不代表转角精度已通过。'])
            elif s.get('mode')=='route_l':
                lines.append('L形组合完成段数：'+str(sum(bool(x['result'].get('ok') and x['result'].get('stop_acknowledged')) for x in s.get('steps',[])))+'/3')
                for step in s.get('steps',[]):
                    item=step['result']
                    value=('陀螺转角 '+str(item.get('gyro_turn_degrees'))+' deg' if item.get('mode')=='turn_relative' else '编码里程 '+str(item.get('travelled_m'))+' m')
                    lines.append(str(step['index'])+'. '+step['label']+'：'+value+'；STOP '+str(item.get('stop_acknowledged',False)))
                if s.get('error'):lines.append('组合中止：'+s['error'])
            elif s.get('mode')=='turn_relative':
                lines.extend(['目标转角：'+str(s.get('target_degrees'))+' deg',
                              '融合航向累计转角（诊断）：'+str(s.get('imu_turn_degrees'))+' deg',
                              '陀螺积分转角：'+str(s.get('gyro_turn_degrees'))+' deg',
                              '控制反馈：'+str(s.get('turn_heading_source','fused_yaw')),
                              '控制转角误差：'+str(s.get('control_angle_error_deg',s.get('final_angle_error_deg')))+' deg',
                              'FPGA判定转角完成：'+str(s.get('completed_by_fpga',False))])
            else:
                lines.extend(['编码器推算距离（非实测车身位移）：'+str(s.get('travelled_m'))+' m',
                              '直线控制反馈：'+str(s.get('straight_heading_source','fused_yaw')),
                              '控制用相对航向误差：'+str(s.get('final_heading_error_deg',s.get('final_yaw_error_deg')))+' deg',
                              '九轴融合航向变化（单独记录）：'+str(s.get('final_yaw_error_deg'))+' deg'])
            if s.get('command_active_seconds') is not None:
                lines.extend(['直线指令阶段：'+str(s['command_active_seconds'])+' s',
                              '等待双轮编码器起转：'+str(s.get('encoder_start_delay_seconds'))+' s',
                              '起转后至STOP：'+str(s.get('encoder_start_to_stop_seconds'))+' s',
                              '停车原因：'+str(s.get('observed_stop_reason'))])
        lines.extend(['车身实际运动和方向，需要你观察确认。','请回到对话告诉我实物结果，或提供本文件。'])
        (session_dir/'RESULT.txt').write_text('\n'.join(lines)+'\n',encoding='utf-8')
        log('\n'.join(lines));log('结果目录：'+str(session_dir))
        if not result['restored']:log('请手动连回原Wi-Fi后再返回对话。')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    mode=p.add_mutually_exclusive_group()
    mode.add_argument('--stop-only',action='store_true')
    mode.add_argument('--turn-degrees',type=int,choices=(-90,-30,30,90))
    mode.add_argument('--observe-still',action='store_true')
    mode.add_argument('--route-l',action='store_true')
    mode.add_argument('--straight-seconds',type=int,choices=(3,6),default=3)
    p.add_argument('--dry-run',action='store_true',help='Show the plan without changing Wi-Fi or contacting the car')
    a=p.parse_args()
    if a.dry_run:
        print('DRY RUN: no network changes and no car commands.')
        if a.route_l:
            print('Connect -> STOP/preflight -> operator GO once -> straight20rpm/6s -> STOP -> left90deg/max10s -> STOP -> straight20rpm/6s -> STOP -> restore Wi-Fi; abort remaining steps on failure')
            return 0
        print('STOP only' if a.stop_only else 'Connect -> STOP -> operator READ -> 12s stationary telemetry, STOP commands only -> STOP -> restore Wi-Fi -> logs' if a.observe_still else 'Connect -> STOP/preflight -> operator GO -> countdown -> '+('turn '+str(a.turn_degrees)+'deg, max 10s' if a.turn_degrees is not None else '20 rpm/'+str(a.straight_seconds)+'s, 0.5m travel guard')+' -> STOP -> restore Wi-Fi -> logs')
        return 0
    if sys.platform!='win32':p.error('This entry point is for Windows')
    session_dir=ROOT/'build/output'/('user_ground_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
    r=execute_session(session_dir,a.stop_only,turn_degrees=a.turn_degrees,observe_still=a.observe_still,straight_seconds=a.straight_seconds,route_l=a.route_l)
    return 0 if r['ok'] or (r.get('cancelled') and r['restored']) else 1


if __name__=='__main__':raise SystemExit(main())
