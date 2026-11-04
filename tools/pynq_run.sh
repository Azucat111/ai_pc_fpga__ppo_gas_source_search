#!/usr/bin/env sh
# Use the same XRT/Python environment as the board's Jupyter service, under sudo.
set -eu
CAR_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
if [ -z "${XILINX_XRT:-}" ] && [ -e /usr/lib/libxrt_core.so ]; then
    export XILINX_XRT=/usr
fi
CAR_PYTHON=python3
if [ -x /usr/local/share/pynq-venv/bin/python3 ]; then
    CAR_PYTHON=/usr/local/share/pynq-venv/bin/python3
fi
export PYTHONPATH="$CAR_ROOT/software${PYTHONPATH:+:$PYTHONPATH}"
cd "$CAR_ROOT"
if [ "$#" -eq 0 ]; then set -- -m car.diagnose --seconds 10; fi
exec "$CAR_PYTHON" "$@"
