#!/usr/bin/env sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cc -std=c99 -O2 -Wall -Wextra -fPIC -shared \
  -I"$ROOT/vendor/bme68x" "$ROOT/software/bme_shim.c" \
  "$ROOT/vendor/bme68x/bme68x.c" -lm -o "$ROOT/software/car/libbme68x_car.so"
printf '%s\n' "Built software/car/libbme68x_car.so for this machine"
