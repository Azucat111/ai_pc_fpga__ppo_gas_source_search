"""Safety arbitration for task-level car control.

The FPGA watchdog remains the last-resort stop mechanism.  This controller
owns the PC-side task state and never turns an unavailable diagnostic field
into a fabricated healthy value.
"""

from enum import Enum
import time


class SafetyState(str, Enum):
    RUNNING = 'running'
    PAUSED = 'paused'
    STOPPED = 'stopped'
    LATCHED_FAULT = 'latched_fault'


class SafetyController:
    """Latched safety state with explicit human recovery."""

    def __init__(self, require_position=False, telemetry_timeout_s=0.6):
        self.require_position = bool(require_position)
        self.telemetry_timeout_s = float(telemetry_timeout_s)
        self.state = SafetyState.RUNNING
        self.fault_code = None
        self.fault_detail = None
        self.last_frame_monotonic = None
        self._stop_request = False
        self.field_availability = {
            'emergency_stop': False,
            'motor_driver_fault': False,
        }

    def observe_frame(self, frame, now=None):
        now = time.monotonic() if now is None else float(now)
        self.last_frame_monotonic = now
        for field in self.field_availability:
            self.field_availability[field] = field in frame
        if frame.get('emergency_stop') is True:
            self.latch('emergency_stop', 'board emergency_stop is asserted')
        elif frame.get('motor_driver_fault') is True:
            self.latch('motor_driver_fault', 'motor driver fault reported by board')
        return self.status()

    def observe_position(self, position):
        if self.require_position and not position.get('valid', False):
            self.latch('position_invalid', 'relative pose is unavailable')
        return self.status()

    def check_telemetry(self, now=None):
        now = time.monotonic() if now is None else float(now)
        if (self.last_frame_monotonic is not None and
                now - self.last_frame_monotonic > self.telemetry_timeout_s):
            self.latch('communication_timeout', 'telemetry exceeded safety timeout')
        return self.status()

    def latch(self, code, detail=None):
        if self.state != SafetyState.LATCHED_FAULT:
            self.state = SafetyState.LATCHED_FAULT
            self.fault_code = str(code)
            self.fault_detail = detail
            self._stop_request = True

    def request_pause(self):
        if self.state == SafetyState.RUNNING:
            self.state = SafetyState.PAUSED
            self._stop_request = True

    def request_stop(self):
        if self.state in (SafetyState.RUNNING, SafetyState.PAUSED):
            self.state = SafetyState.STOPPED
            self._stop_request = True

    def resume(self):
        """Resume only an intentional pause; faults require reset_fault."""
        if self.state == SafetyState.PAUSED:
            self.state = SafetyState.RUNNING
            return True
        return False

    def reset_fault(self, frame=None, position=None):
        """Clear a latched fault only after the operator explicitly resets it."""
        if self.state != SafetyState.LATCHED_FAULT:
            return False
        if frame is not None:
            if frame.get('emergency_stop') is True or frame.get('motor_driver_fault') is True:
                return False
        if self.require_position and position is not None and not position.get('valid', False):
            return False
        self.state = SafetyState.RUNNING
        self.fault_code = None
        self.fault_detail = None
        self._stop_request = True
        return True

    def consume_stop_request(self):
        requested = self._stop_request
        self._stop_request = False
        return requested

    def action_allowed(self):
        return self.state == SafetyState.RUNNING

    def status(self):
        return {
            'state': self.state.value,
            'fault_code': self.fault_code,
            'fault_detail': self.fault_detail,
            'field_availability': dict(self.field_availability),
            'telemetry_timeout_s': self.telemetry_timeout_s,
        }
