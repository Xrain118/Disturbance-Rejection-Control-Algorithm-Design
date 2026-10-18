#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/env.sh"
: "${FIRMWARE_ROOT:?Set path to the external RoArm-M3_example}"
: "${ARDUINOJSON_ROOT:?Set path to installed ArduinoJson library}"
output="${FIRMWARE_TEST_OUTPUT:-$PROJECT_ROOT/log/firmware-tests}"
mkdir -p "$output"
tests="$PROJECT_ROOT/src/dragonfly_serial/test"
g++ -std=c++17 -O2 -pthread -I"$FIRMWARE_ROOT" -I"$tests/mock_platform" "$tests/firmware_concurrency.cpp" -o "$output/concurrency"
"$output/concurrency"
g++ -std=c++17 -O2 -I"$FIRMWARE_ROOT" -I"$ARDUINOJSON_ROOT/src" "$tests/firmware_contract.cpp" -o "$output/contract"
"$output/contract" "$tests/data" "$output/firmware-frames.jsonl"
PYTHONPATH="$PROJECT_ROOT/src/dragonfly_serial" python3 - "$output/firmware-frames.jsonl" "$FIRMWARE_ROOT" <<'PY'
import sys
from dragonfly_serial.protocol import parse
with open(sys.argv[1]) as f:
    for line in f: parse(line.strip())
print('Host parser accepted both actual firmware-generated frames')
from pathlib import Path
import re
root=Path(sys.argv[2])
for source in list(root.glob('*.h'))+list(root.glob('*.ino')):
    if source.name not in ('telemetry_platform.h','RoArm-M3_example.ino'):
        assert not re.search(r'\bSerial\b',source.read_text()), source.name+' bypasses unified output'
esp=(root/'esp_now_ctrl.h').read_text()
callback=esp.split('void OnDataRecv(',1)[1].split('void processEspNowCommands()',1)[0]
assert 'RoArmM3_' not in callback and 'jsonCmdReceive' not in callback
print('Application UART output and ESP-NOW callback isolation audit passed')
PY
