"""Clear motor permission on an existing car core without loading/configuring hardware."""
import json
import time
from .debug import read_diagnostics


def disable_motion(mmio):
    if mmio.read(0)!=0x43415231:raise RuntimeError('CAR1 signature absent; no registers were written')
    mmio.write(0x14,0)
    deadline=time.monotonic()+.25
    while True:
        state=read_diagnostics(mmio)
        if (not state['motor_permission'] and not state['drive_active']
                and not any(state['duty_permille'].values()) and not any(state['output_logic'].values())):
            return state
        if time.monotonic()>=deadline:raise RuntimeError('Motor disable did not reach zero outputs')
        time.sleep(.01)


def main():
    from pynq import MMIO
    print(json.dumps(disable_motion(MMIO(0x43c00000,0x1000))),flush=True)
    return 0


if __name__=='__main__':raise SystemExit(main())
