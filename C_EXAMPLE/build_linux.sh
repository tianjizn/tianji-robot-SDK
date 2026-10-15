#!/bin/bash
# ==========================================================================
#  Build one (or all) C_EXAMPLE example(s) -- Method 1:
#  compile the example directly with all C_SDK source (no prebuilt library).
#
#  Usage:
#    ./build_linux.sh                          -> builds example_basic_LinkSystem
#    ./build_linux.sh example_basic_Position
#    ./build_linux.sh example_basic_Position.cpp   (.cpp optional)
#    ./build_linux.sh all                      -> builds every example_*.cpp
#
#  Output: <name> next to each source file. Run it yourself.
# ==========================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SDK_DIR="$SCRIPT_DIR/../C_SDK"

# All C_SDK sources (bash expands the *.cpp globs).
SDK_CPP="$SDK_DIR/FXUtility/FXMath/*.cpp $SDK_DIR/FXUtility/FXCfg/*.cpp $SDK_DIR/Interf/*.cpp $SDK_DIR/Kinematics/*.cpp $SDK_DIR/Kinematics/ArmKinematics/*.cpp $SDK_DIR/Kinematics/DynaIdent/*.cpp $SDK_DIR/Kinematics/KineCommon/*.cpp $SDK_DIR/Kinematics/MotionPlanner/*.cpp $SDK_DIR/Kinematics/SkyeBodyKinematics/*.cpp $SDK_DIR/FileClient/*.cpp $SDK_DIR/L0Control/*.cpp $SDK_DIR/L1Robot/*.cpp"

# SDK include dirs (same set as ../linux_auto_compile.sh).
INC_DIRS="-I$SDK_DIR/Common -I$SDK_DIR/FXUtility -I$SDK_DIR/FXUtility/FXMath -I$SDK_DIR/FXUtility/FXCfg -I$SDK_DIR/Interf -I$SDK_DIR/Kinematics -I$SDK_DIR/Kinematics/ArmKinematics -I$SDK_DIR/Kinematics/DynaIdent -I$SDK_DIR/Kinematics/KineCommon -I$SDK_DIR/Kinematics/MotionPlanner -I$SDK_DIR/Kinematics/SkyeBodyKinematics -I$SDK_DIR/FileClient -I$SDK_DIR/L0Control -I$SDK_DIR/L1Robot"

build_one() {
    local n="$1"
    if [ ! -f "$SCRIPT_DIR/$n.cpp" ]; then
        echo "[FAIL] Not found: $SCRIPT_DIR/$n.cpp"
        return 1
    fi
    echo "--------------------------------------------"
    echo "Building $n.cpp  (Method 1: compile with C_SDK source)"
    echo "--------------------------------------------"
    g++ -w "$SCRIPT_DIR/$n.cpp" $SDK_CPP $INC_DIRS -o "$SCRIPT_DIR/$n" -lrt -DCMPL_LIN -DL1_SDK_EXPORTS
    echo "[OK] $n    Run it: $SCRIPT_DIR/$n"
}

NAME="${1:-example_basic_LinkSystem}"
NAME="${NAME%.cpp}"   # strip optional .cpp

if [ "$NAME" = "all" ]; then
    FAIL=0
    for f in "$SCRIPT_DIR"/example_*.cpp; do
        n="$(basename "$f" .cpp)"
        build_one "$n" || FAIL=1
    done
    echo ""
    if [ "$FAIL" = "1" ]; then echo "[DONE] Some examples FAILED."; else echo "[DONE] All examples built OK."; fi
    exit "$FAIL"
fi

build_one "$NAME"
