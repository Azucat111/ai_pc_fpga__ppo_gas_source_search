"""Original GUI with temporary Windows wireless performance mode; no extra control client."""
import sys
import windows_remote
from wifi_power_session import performance_mode,restore_power

def main():
    if '--smoke-test' in sys.argv:return windows_remote.main()
    saved=None
    try:
        saved=performance_mode()
        print('Wireless performance mode enabled for this remote session; restored on normal exit.',flush=True)
    except Exception as exc:
        print('Wireless power mode unchanged/unconfirmed:',exc,flush=True)
    try:return windows_remote.main()
    finally:
        if saved:
            try:restore_power(saved)
            except Exception as exc:
                print('Wireless power setting restore failed; original DC index:',saved['dc'],str(exc),flush=True)
                raise
if __name__=='__main__':main()
