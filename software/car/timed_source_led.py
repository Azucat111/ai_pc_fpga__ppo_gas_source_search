"""Nonblocking timed LED session on the existing telemetry/control socket."""
import time
from .source_event import SourceIndicator, QUERY, SOURCE_CONFIRMED, SOURCE_CLEARED


class TimedSourceLED:
    def __init__(self, client, notify):
        self.indicator=SourceIndicator(client);self.notify=notify
        self.phase='idle';self.maybe_lit=False;self.deadline=0;self.retry_at=0

    @property
    def active(self):return self.phase!='idle'

    def start(self):
        if self.active:
            self.notify('LD5正在处理，请等待本次结束');return
        try:
            self.indicator.request(QUERY);self.phase='query'
            self.notify('LD5正在连接')
        except (OSError,RuntimeError) as exc:self.notify('LD5：'+str(exc))

    def cancel(self):
        # Do not leave a confirmation queued after the window closes.
        self.indicator.pending=None
        if self.maybe_lit:self.phase='recover';self.retry_at=0
        else:self.phase='idle'

    def poll(self):
        if not self.active:return
        try:
            if self.phase=='recover':
                if time.monotonic()<self.retry_at:return
                self.indicator.request(QUERY);self.phase='clear_query'
            result=self.indicator.poll()
            if self.phase=='query' and result is not None:
                self.maybe_lit=True
                self.indicator.request(SOURCE_CONFIRMED)
                self.deadline=time.monotonic()+5;self.phase='on'
            elif self.phase=='on' and result is not None:
                self.phase='flashing';self.notify('LD5蓝灯闪烁中，5秒后关闭')
            elif self.phase=='flashing' and time.monotonic()>=self.deadline:
                self.indicator.request(SOURCE_CLEARED);self.phase='off'
                self.notify('LD5正在关闭')
            elif self.phase=='clear_query' and result is not None:
                self.indicator.request(SOURCE_CLEARED);self.phase='off'
            elif self.phase=='off' and result is not None:
                if result['confirmed']:raise RuntimeError('清除应答仍为亮灯')
                self.maybe_lit=False;self.phase='idle';self.notify('LD5已关闭')
        except (OSError,RuntimeError) as exc:
            self.indicator.pending=None
            self.phase='recover' if self.maybe_lit else 'idle'
            self.retry_at=time.monotonic()+1
            self.notify('LD5：'+str(exc)+('；恢复连接后继续清除' if self.maybe_lit else ''))
