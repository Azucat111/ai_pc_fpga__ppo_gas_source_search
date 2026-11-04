"""Windows Tk remote. Starting the window registers telemetry only."""
import argparse
from pathlib import Path
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'software'))
from car.config import load_config
from car.host import Client,ControlBusy
from car.teleop import Teleop,STRAIGHT_RPM,TURN_RPM
from car.timed_source_led import TimedSourceLED


class Intent:
    def __init__(self):
        self.lock=threading.Lock();self.direction=None;self.revision=0;self.alive=time.monotonic();self.force=False;self.led_requested=False
    def request_led(self):
        with self.lock:self.led_requested=True
    def take_led(self):
        with self.lock:
            requested=self.led_requested;self.led_requested=False;return requested
    def set(self,direction,force=False):
        with self.lock:
            if direction!=self.direction or force:
                self.direction=direction;self.force=force;self.revision+=1
    def snapshot(self):
        with self.lock:return self.direction,self.revision,self.alive,self.force
    def pulse(self):
        with self.lock:self.alive=time.monotonic()


def worker(intent,events,closing,ip):
    client=Client(ip,load_config(ROOT/'config/car.json'));engine=Teleop(client)
    led=TimedSourceLED(client,lambda message:events.put(('led',message)))
    seen=-1;desired=None;blocked=False;last_status=0
    try:
        while not closing.is_set():
            direction,revision,alive,force=intent.snapshot()
            if revision!=seen:
                seen=revision;blocked=False;desired=direction
                if engine.owns or force:engine.stop(force=force)
            if time.monotonic()-alive>.25:
                desired=None;blocked=True
                if engine.phase in ('starting','driving'):engine.stop()
            try:
                state=engine.poll()
                if desired and not blocked and engine.phase=='idle':engine.start(desired)
                if state and time.monotonic()-last_status>.1:
                    last_status=time.monotonic();events.put(('state',state,engine.phase))
                if time.monotonic()-client.received_at>1:
                    events.put(('status','未连接：请连接 FPGA-CAR Wi-Fi，并等待小车启动'))
            except (OSError,RuntimeError,TimeoutError) as exc:
                desired=None;blocked=True
                if engine.owns:
                    try:engine.stop()
                    except (OSError,RuntimeError):pass
                    # A STOP datagram was sent; cease all motion keepalives.
                    engine.abandon()
                events.put(('fault',str(exc)))
            if intent.take_led():led.start()
            led.poll()
    except Exception as exc:
        events.put(('fault',str(exc)))
    finally:
        if engine.owns:
            try:
                engine.stop();deadline=time.monotonic()+.45
                while engine.owns and time.monotonic()<deadline:engine.poll()
            except (OSError,RuntimeError,TimeoutError):pass
        led.cancel();deadline=time.monotonic()+.8
        while led.active and time.monotonic()<deadline:
            try:client.receive(timeout=.02)
            except (OSError,RuntimeError):pass
            led.poll()
        if led.maybe_lit:print('LD5清除未确认：请恢复连接后重新点击LED按钮，或重启小车。',flush=True)
        client.close()


class Window:
    def __init__(self,root,smoke=False):
        self.root=root;self.intent=Intent();self.events=queue.Queue();self.closing=threading.Event()
        self.thread=None;self.current=None;self.releases={};self.last_state=None;self.updated=0;self.mouse_held=False
        root.title('FPGA 小车 · Windows 遥控');root.geometry('650x650');root.minsize(600,620)
        frame=ttk.Frame(root,padding=22);frame.pack(fill='both',expand=True)
        ttk.Label(frame,text='小车遥控',font=('Microsoft YaHei UI',22,'bold')).pack(anchor='w')
        ttk.Label(frame,text='W / S 前后 · A / D 原地转向 · 松开停车 · 空格停止',font=('Microsoft YaHei UI',11)).pack(anchor='w',pady=(8,4))
        ttk.Label(frame,text=f'直行 {STRAIGHT_RPM} rpm / 转向 {TURN_RPM} rpm；按键或鼠标按钮均需按住。').pack(anchor='w')
        self.status=tk.StringVar(value='等待连接 FPGA-CAR Wi-Fi；窗口启动不会发运动指令')
        ttk.Label(frame,textvariable=self.status,wraplength=590,foreground='#195b88').pack(anchor='w',pady=14)
        pad=ttk.Frame(frame);pad.pack(pady=5)
        for key,label,row,col in [('w','↑ 前进 (W)',0,1),('a','↶ 左转 (A)',1,0),('s','↓ 后退 (S)',1,1),('d','↷ 右转 (D)',1,2)]:
            button=ttk.Button(pad,text=label,width=18)
            button.grid(row=row,column=col,padx=5,pady=5,ipady=14)
            button.bind('<ButtonPress-1>',lambda event,k=key:self.press_mouse(k))
        ttk.Button(frame,text='停止 · Space / Esc',command=self.stop).pack(fill='x',pady=12,ipady=8)
        self.led_status=tk.StringVar(value='LD5蓝灯：待命')
        ttk.Button(frame,text='LD5蓝灯闪烁5秒',command=self.intent.request_led).pack(fill='x',ipady=5)
        ttk.Label(frame,textvariable=self.led_status,wraplength=590).pack(anchor='w',pady=5)
        self.feedback=tk.StringVar(value='轮速 —    |    IMU —')
        self.sensors=tk.StringVar(value='温湿度 / 气压 / TGS：等待数据')
        ttk.Label(frame,textvariable=self.feedback,wraplength=590).pack(anchor='w',pady=8)
        ttk.Label(frame,textvariable=self.sensors,wraplength=590).pack(anchor='w',pady=8)
        ttk.Label(frame,text='两台上位机可观察、轮流控制；对方行驶时会提示占用。\n每台电脑仅运行一个小车程序。连接热点后本机可能暂时无法上网。',
                  wraplength=590,foreground='#666666').pack(anchor='w',pady=10)
        root.bind('<KeyPress>',self.key_down);root.bind('<KeyRelease>',self.key_up)
        root.bind_all('<ButtonRelease-1>',lambda event:self.release_mouse())
        root.bind('<FocusOut>',lambda event:root.after(10,self.focus_check))
        root.bind('<Unmap>',lambda event:self.release())
        root.protocol('WM_DELETE_WINDOW',self.close)
        if not smoke:
            self.thread=threading.Thread(target=worker,args=(self.intent,self.events,self.closing,'192.168.4.1'),daemon=True)
            self.thread.start();root.after(40,self.tick)

    def press(self,key):
        if key in self.releases:self.root.after_cancel(self.releases.pop(key))
        if self.current!=key:
            self.current=key;self.intent.set(key)

    def release(self):
        self.current=None;self.intent.set(None)

    def press_mouse(self,key):
        self.mouse_held=True;self.press(key)

    def release_mouse(self):
        if self.mouse_held:
            self.mouse_held=False;self.release()

    def stop(self):
        self.current=None;self.intent.set(None,force=True);self.status.set('已请求停止')

    def key_down(self,event):
        key=event.keysym.lower()
        if key in ('space','escape'):self.stop();return 'break'
        if key in 'wasd' and len(key)==1:self.press(key);return 'break'

    def key_up(self,event):
        key=event.keysym.lower()
        if key in 'wasd' and len(key)==1:
            if key in self.releases:self.root.after_cancel(self.releases[key])
            def released():
                self.releases.pop(key,None)
                if self.current==key:self.release()
            # X11/Tk and some Windows keyboards synthesize repeat release events.
            self.releases[key]=self.root.after(35,released)
            return 'break'

    def focus_check(self):
        if self.root.focus_get() is None:self.release()

    def tick(self):
        self.intent.pulse()
        while True:
            try:item=self.events.get_nowait()
            except queue.Empty:break
            if item[0]=='state':
                s,phase=item[1:];self.updated=time.monotonic();self.last_state=s
                label={'starting':'请求控制中','driving':'本机正在控制','stopping':'正在停车','idle':'已连接 · 停车待命' if s['last_action']=='stop' else '已连接 · 另一台正在控制'}[phase]
                if self.current is None or phase!='idle':self.status.set(label)
                self.feedback.set(f"轮速 左 {s['wheel_rpm_left']:.1f} / 右 {s['wheel_rpm_right']:.1f} rpm   |   IMU {s['imu_age_ms']} ms   |   boot {s['boot_id']}")
                b=s['bme688'];t=s['tgs2602']
                def fmt(value,digits=1):return '—' if value is None else f'{value:.{digits}f}'
                self.sensors.set(f"温度 {fmt(b['temperature_c'])} ℃   湿度 {fmt(b['humidity_percent'])} %   气压 {fmt(b['pressure_pa'],0)} Pa\nTGS 电压 {fmt(t[0]['voltage_v'],3)} / {fmt(t[1]['voltage_v'],3)} V")
            elif item[0]=='led':self.led_status.set(item[1])
            elif item[0]=='fault':
                self.release();self.status.set(item[1]+'；松开后重新按键')
            else:self.status.set(item[1])
        if self.updated and time.monotonic()-self.updated>1:self.status.set('连接中断；松开按键，恢复连接后重新操作')
        if not self.closing.is_set():self.root.after(40,self.tick)

    def close(self):
        self.release();self.closing.set();self.root.destroy()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--smoke-test',action='store_true');args=parser.parse_args()
    root=tk.Tk();window=Window(root,smoke=args.smoke_test)
    if args.smoke_test:
        root.update();assert window.thread is None;root.destroy();print('PASS remote window smoke test: no networking or motion');return
    try:root.mainloop()
    finally:
        window.closing.set()
        if window.thread:window.thread.join(timeout=2)

if __name__=='__main__':main()
