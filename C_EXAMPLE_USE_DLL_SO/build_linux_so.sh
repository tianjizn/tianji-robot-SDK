#!/bin/bash
# ==========================================================================
#  Build test code against the prebuilt libGentoSDK.so -- Method 2.
#  The SO is produced by ../linux_auto_compile.sh and lives in this folder.
#
#  Usage:
#    ./build_linux_so.sh                        -> builds local test_link.cpp
#    ./build_linux_so.sh example_basic_Position
#    ./build_linux_so.sh example_basic_Position.cpp   (.cpp optional)
#    ./build_linux_so.sh all                    -> builds every ../C_EXAMPLE/example_*.cpp
#
#  Example sources are read from ../C_EXAMPLE/. Output <name> lands here.
#  Run the executable yourself.
# ==========================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SDK_DIR="$SCRIPT_DIR/../C_SDK"
EX_DIR="$SCRIPT_DIR/../C_EXAMPLE"

INC_DIRS="-I$SDK_DIR/Common -I$SDK_DIR/FXUtility -I$SDK_DIR/FXUtility/FXMath -I$SDK_DIR/FXUtility/FXCfg -I$SDK_DIR/Interf -I$SDK_DIR/Kinematics -I$SDK_DIR/Kinematics/ArmKinematics -I$SDK_DIR/Kinematics/DynaIdent -I$SDK_DIR/Kinematics/KineCommon -I$SDK_DIR/Kinematics/MotionPlanner -I$SDK_DIR/Kinematics/SkyeBodyKinematics -I$SDK_DIR/FileClient -I$SDK_DIR/L0Control -I$SDK_DIR/L1Robot"

echo "============================================"
echo "Checking compiled library"
echo "============================================"
if [ ! -f "$SCRIPT_DIR/libGentoSDK.so" ]; then
    echo "[FAIL] libGentoSDK.so not found in $SCRIPT_DIR"
    echo "       Run ../linux_auto_compile.sh first to compile the SDK."
    exit 1
fi
echo "[OK] libGentoSDK.so found."

build_one() {
    local name="$1" src="$2" rc=0
    if [ ! -f "$src" ]; then
        echo "[FAIL] Not found: $src"
        return 1
    fi
    echo "--------------------------------------------"
    echo "Building $name.cpp  (Method 2: link prebuilt SO)"
    echo "--------------------------------------------"
    # -include FXCmplOpt.h : pull socket/pthread/atomic in up front.
    # -pthread             : the examples use std::thread.
    # -Wl,-rpath,'$ORIGIN' : find libGentoSDK.so next to the executable.
    g++ -include FXCmplOpt.h "$src" $INC_DIRS \
        -L"$SCRIPT_DIR" -lGentoSDK -Wl,-rpath,'$ORIGIN' -pthread \
        -DCMPL_LIN -Wall -O2 \
        -o "$SCRIPT_DIR/$name" || rc=$?
    if [ "$rc" -ne 0 ]; then
        echo "[FAIL] $name build failed (rc=$rc)"
    else
        echo "[OK] $name    Run: $SCRIPT_DIR/$name"
    fi
    return "$rc"
}

NAME="${1:-test_link}"
NAME="${NAME%.cpp}"   # strip optional .cpp

if [ "$NAME" = "all" ]; then
    echo "Building every example_*.cpp from $EX_DIR (Method 2: link prebuilt SO)"
    FAIL=0
    for f in "$EX_DIR"/example_*.cpp; do
        n="$(basename "$f" .cpp)"
        build_one "$n" "$f" || FAIL=1
    done
    echo ""
    if [ "$FAIL" = "1" ]; then echo "[DONE] Some examples FAILED."; else echo "[DONE] All examples built OK."; fi
    exit "$FAIL"
fi

# resolve source: this folder first, then ../C_EXAMPLE
if [ -f "$SCRIPT_DIR/$NAME.cpp" ]; then SRC="$SCRIPT_DIR/$NAME.cpp"; else SRC="$EX_DIR/$NAME.cpp"; fi
build_one "$NAME" "$SRC"
exit $?
