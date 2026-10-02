#!/bin/bash

# ── SAR B200 Run Script ──────────────────────────────────────────────
# Usage: ./run.sh
# Requires: sudo, UHD 4.8, B200 connected via USB 3.0
# chmod +x run.sh
# ./run.sh
# ─────────────────────────────────────────────────────────────────────

set -e  # exit on error

# ── Paths ─────────────────────────────────────────────────────────────
BINARY="./connect"
UHD_IMAGES="/usr/share/uhd/4.8.0/images"
DATA_DIR="Data"

# ── Checks ────────────────────────────────────────────────────────────
echo "[Run] Checking binary..."
if [ ! -f "$BINARY" ]; then
    echo "[Run] Binary not found — building..."
    make clean && make
fi

echo "[Run] Checking UHD images..."
if [ ! -f "$UHD_IMAGES/usrp_b200_fw.hex" ]; then
    echo "[ERROR] UHD images not found at $UHD_IMAGES"
    exit 1
fi

echo "[Run] Checking B200 connection..."
if ! uhd_find_devices 2>/dev/null | grep -q "B200"; then
    echo "[ERROR] B200 not detected — check USB connection"
    exit 1
fi

echo "[Run] Checking Data directory..."
mkdir -p "$DATA_DIR"

echo "[Run] Checking isolated CPUs..."
ISOLATED=$(cat /sys/devices/system/cpu/isolated 2>/dev/null || echo "none")
echo "[Run] Isolated cores: $ISOLATED"

# ── Run ───────────────────────────────────────────────────────────────
echo ""
echo "[Run] ────────────────────────────────────────────"
echo "[Run]  SAR Session Starting"
echo "[Run]  Binary : $BINARY"
echo "[Run]  Images : $UHD_IMAGES"
echo "[Run]  Data   : $DATA_DIR"
echo "[Run]  Time   : $(date '+%Y-%m-%d %H:%M:%S')"
echo "[Run] ────────────────────────────────────────────"
echo ""

sudo UHD_IMAGES_DIR="$UHD_IMAGES" "$BINARY"

echo ""
echo "[Run] ────────────────────────────────────────────"
echo "[Run]  Session complete"
echo "[Run]  Data saved to: $DATA_DIR/"
ls -lh "$DATA_DIR"/rx_data_*.bin 2>/dev/null || echo "[Run]  No data files found"
echo "[Run] ────────────────────────────────────────────"