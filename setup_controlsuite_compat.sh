#!/bin/bash
# Create a controlSUITE-compatible directory tree using symlinks into C2000Ware.
#
# The MATLAB C2000 build system for older processors (F2806x etc.) expects paths like:
#   $(CONTROLSUITEINSTALLDIR)/libs/math/IQmath/v160/include
#   $(CONTROLSUITEINSTALLDIR)/libs/math/IQmath/v160/lib
#   $(CONTROLSUITEINSTALLDIR)/libs/math/CLAmath/v400/include
#   $(CONTROLSUITEINSTALLDIR)/libs/utilities/hrcap_hccal/type0/v110/include
#
# C2000Ware (the successor to ControlSUITE) has them at different paths:
#   C2000Ware/libraries/math/IQmath/c28/{include,lib}
#   C2000Ware/libraries/math/CLAmath/c28/include
#   C2000Ware/libraries/calibration/hrcap/common/type0/include
#
# This script creates a bridge directory at C2000Ware/controlsuite_compat/
# that maps the old paths to the new locations via symlinks.
#
# Usage: bash setup_controlsuite_compat.sh

set -euo pipefail

C2000WARE="/home/user/ti/c2000/C2000Ware_4_03_00_00"
COMPAT_ROOT="${C2000WARE}/controlsuite_compat"

if [[ ! -d "$C2000WARE" ]]; then
    echo "ERROR: C2000Ware not found at $C2000WARE"
    exit 1
fi

echo "Creating ControlSUITE compatibility tree at:"
echo "  $COMPAT_ROOT"
echo ""

# ---- IQmath: libs/math/IQmath/v160 -> libraries/math/IQmath/c28 ----
mkdir -p "${COMPAT_ROOT}/libs/math/IQmath"
ln -sfn "${C2000WARE}/libraries/math/IQmath/c28" "${COMPAT_ROOT}/libs/math/IQmath/v160"
echo "[OK] IQmath v160 -> IQmath/c28"

# ---- CLAmath: libs/math/CLAmath/v400 -> libraries/math/CLAmath/c28 ----
mkdir -p "${COMPAT_ROOT}/libs/math/CLAmath"
ln -sfn "${C2000WARE}/libraries/math/CLAmath/c28" "${COMPAT_ROOT}/libs/math/CLAmath/v400"
echo "[OK] CLAmath v400 -> CLAmath/c28"

# ---- HRCAP: libs/utilities/hrcap_hccal/type0/v110 -> calibration/hrcap/common/type0 ----
mkdir -p "${COMPAT_ROOT}/libs/utilities/hrcap_hccal/type0"
ln -sfn "${C2000WARE}/libraries/calibration/hrcap/common/type0" "${COMPAT_ROOT}/libs/utilities/hrcap_hccal/type0/v110"
echo "[OK] HRCAP v110 -> calibration/hrcap/common/type0"

echo ""
echo "Verification:"
for p in \
    "${COMPAT_ROOT}/libs/math/IQmath/v160/include" \
    "${COMPAT_ROOT}/libs/math/IQmath/v160/lib" \
    "${COMPAT_ROOT}/libs/math/CLAmath/v400/include" \
    "${COMPAT_ROOT}/libs/utilities/hrcap_hccal/type0/v110/include"; do
    if [[ -d "$p" ]]; then
        echo "  [OK] $p"
    else
        echo "  [FAIL] $p"
    fi
done

echo ""
echo "Done. Now run register_c2000_tools in MATLAB."
