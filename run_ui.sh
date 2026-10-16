#!/usr/bin/env bash
# Run the UI Visualization Client
cd "$(dirname "$0")"
BIN=./build/docking_ui
if [[ ! -x $BIN ]]; then
    echo "Please build the project first: cmake -B build && cmake --build build -j" >&2
    exit 1
fi
echo "[RUN] Starting Spacecraft Docking UI..."
exec "$BIN" "$@"
