#!/usr/bin/env bash
# Run the Physics Engine Server
cd "$(dirname "$0")"
BIN=./build/physics_engine

if [[ ! -x $BIN ]]; then
    echo "Please build the project first: cmake -B build && cmake --build build -j" >&2
    exit 1
fi

echo "[RUN] Starting Physics Engine Server..."
exec "$BIN" "$@"
