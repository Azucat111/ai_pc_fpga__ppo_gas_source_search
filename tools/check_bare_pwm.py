"""Isolated PYNQ-Z2 base-overlay AXI timer PWM check; no car/ESP required.

Uses the Arduino mailbox only to configure hardware, never to time GPIO edges.
Timer setup follows Xilinx embeddedsw xilinx_v2022.2 xtmrctr.c PwmConfigure/
PwmEnable. IO switch PWM0 is distinct from TIMER_G0.
"""
import argparse
import json
from pathlib import Path
import signal
import time
import xml.etree.ElementTree as ET


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bare-board', required=True, action='store_true')
    p.add_argument('--pin', type=int, choices=(3, 5, 6, 9), default=3)
    p.add_argument('--duty', type=int, choices=(25, 50, 75), default=50)
    p.add_argument('--seconds', type=int, default=300)
    args = p.parse_args()
    if not 5 <= args.seconds <= 600:
        p.error('--seconds must be 5..600')
    for f in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            cmd = f.read_bytes()
        except OSError:
            continue
        if any(name in cmd for name in (b'-m\x00car.board', b'-m\x00car.diagnose', b'check_motor_gpio')):
            raise RuntimeError('Conflicting board process: ' + f.parent.name)
    import pynq.overlays.base
    from pynq.overlays.base import BaseOverlay
    from pynq.lib.arduino import Arduino_IO, ARDUINO_DIO_BASEADDR
    from pynq.lib.arduino import ARDUINO_SWITCHCONFIG_BASEADDR, ARDUINO_SWCFG_PWM0

    bit = Path(pynq.overlays.base.__file__).parent / 'base.bit'
    tree = ET.parse(bit.with_suffix('.hwh'))
    module = next(m for m in tree.iter('MODULE')
                  if m.get('FULLNAME') == '/iop_arduino/timers_subsystem/timer_0')
    params = {v.get('NAME'): v.get('VALUE') for v in module.findall('./PARAMETERS/PARAMETER')}
    clock = next(v for v in module.findall('./PORTS/PORT') if v.get('NAME') == 's_axi_aclk')
    hz = int(clock.get('CLKFREQUENCY'))
    timer = int(params['C_BASEADDR'], 0)
    if hz != 100000000 or timer != 0x41c00000 or any(params[k] != '1' for k in ('C_GEN0_ASSERT', 'C_GEN1_ASSERT')):
        raise RuntimeError('Unexpected base timer configuration; no hardware was changed')
    period = hz // 20000
    high = period * args.duty // 100
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    bridge = None
    motor_mask = sum(1 << n for n in (3, 5, 6, 9))
    try:
        overlay = BaseOverlay(str(bit))
        for pin in (3, 5, 6, 9):
            io = Arduino_IO(overlay.ARDUINO, pin, 'out')
            bridge = io.microblaze
            bridge.write_cmd(ARDUINO_DIO_BASEADDR, 0)
        write, read = bridge.write_cmd, bridge.read_cmd
        write(timer, 0)
        write(timer + 0x10, 0)
        write(timer + 4, period - 2)
        write(timer + 0x14, high - 2)
        for offset in (0, 0x10):
            write(timer + offset, 0x236)  # PWM, load, auto reload, generate, down count.
            write(timer + offset, 0x216)  # Release load; keep stopped.
        write(timer, 0x616)  # Enable both counters simultaneously.
        switch = ARDUINO_SWITCHCONFIG_BASEADDR + (args.pin // 4) * 4
        mux = ARDUINO_SWCFG_PWM0 << ((args.pin % 4) * 8)
        write(switch, mux)
        time.sleep(.05)
        regs = {hex(o): read(timer + o) for o in (0, 4, 8, 0x10, 0x14, 0x18)}
        if read(switch) != mux or regs['0x4'] != period - 2 or regs['0x14'] != high - 2:
            raise RuntimeError('Timer load or pin mux readback mismatch')
        if any(regs[hex(o)] & 0x296 != 0x296 for o in (0, 0x10)):
            raise RuntimeError('Timer PWM control readback mismatch')
        print(json.dumps({'ready': True, 'pin': args.pin, 'configured_frequency_hz': hz / period,
                          'configured_duty_percent': args.duty, 'expected_average_v': 3.3 * high / period,
                          'hold_seconds': args.seconds, 'timer_registers': regs,
                          'switch_register': hex(switch), 'switch_value': hex(read(switch)),
                          'external_waveform_verified': False}), flush=True)
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            time.sleep(min(.2, max(0, end - time.monotonic())))
    except KeyboardInterrupt:
        pass
    finally:
        if bridge is not None:
            # Restore GPIO low before stopping timer; no dependence on PWM idle polarity.
            bridge.write_cmd(ARDUINO_DIO_BASEADDR, 0)
            for i in range(5):
                bridge.write_cmd(ARDUINO_SWITCHCONFIG_BASEADDR + 4*i, 0)
            bridge.write_cmd(timer, 0)
            bridge.write_cmd(timer + 0x10, 0)
            print(json.dumps({'finished': True, 'motor_gpio_bits': bridge.read_cmd(ARDUINO_DIO_BASEADDR) & motor_mask,
                              'timer_control': [bridge.read_cmd(timer), bridge.read_cmd(timer + 0x10)],
                              'base_overlay_retained': True}), flush=True)


if __name__ == '__main__':
    main()
