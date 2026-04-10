#!/bin/bash
# launch_matlab_12m.sh — Start MATLAB with 12 Mbaud serial support on Linux
#
# This script:
#   1. Sets LD_PRELOAD to load the baud rate shim (native-level fix)
#   2. Launches MATLAB with the serial_baudrate_patch on the path (MATLAB-level fix)
#
# Usage:
#   ./launch_matlab_12m.sh              # launch MATLAB GUI
#   ./launch_matlab_12m.sh -batch "..."  # run batch command
#   ./launch_matlab_12m.sh -r "..."      # run startup command

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SHIM_LIB="${SCRIPT_DIR}/libbaudrate_shim.so"
PATCH_DIR="${SCRIPT_DIR}/serial_baudrate_patch"
MATLAB_BIN="/home/user/MATLAB/R2023b/bin/matlab"

# Build shim if missing
if [ ! -f "$SHIM_LIB" ]; then
    echo "[launch_matlab_12m] Building libbaudrate_shim.so ..."
    gcc -shared -fPIC -O2 -o "$SHIM_LIB" "${SCRIPT_DIR}/baudrate_shim.c" -ldl
    if [ $? -ne 0 ]; then
        echo "ERROR: Failed to compile baudrate_shim.c" >&2
        exit 1
    fi
fi

# Prepend to LD_PRELOAD (preserve existing)
if [ -n "$LD_PRELOAD" ]; then
    export LD_PRELOAD="${SHIM_LIB}:${LD_PRELOAD}"
else
    export LD_PRELOAD="${SHIM_LIB}"
fi

echo "[launch_matlab_12m] LD_PRELOAD=${LD_PRELOAD}"
echo "[launch_matlab_12m] MATLAB path patch: ${PATCH_DIR}"

# Launch MATLAB, adding the patch directory to the MATLAB path
# Use -r to run addpath before anything else
ADDPATH_CMD="addpath('${PATCH_DIR}');"

if [ $# -eq 0 ]; then
    # No arguments — GUI mode with path patch
    exec "$MATLAB_BIN" -r "$ADDPATH_CMD"
else
    # Pass-through arguments; inject addpath if -r or -batch is used
    case "$1" in
        -r)
            shift
            exec "$MATLAB_BIN" -r "${ADDPATH_CMD} $*"
            ;;
        -batch)
            shift
            exec "$MATLAB_BIN" -batch "${ADDPATH_CMD} $*"
            ;;
        *)
            exec "$MATLAB_BIN" "$@"
            ;;
    esac
fi
