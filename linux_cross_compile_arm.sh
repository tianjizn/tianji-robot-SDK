#!/bin/bash
#
# Cross-compile the GENTO SDK shared libraries for ARM on an x86_64 Ubuntu 20.04 host.
#
# Usage:
#   ./linux_cross_compile_arm.sh [aarch64|armhf|armel|all]   (default: aarch64)
#
# Prerequisites (install once on the x86_64 build host):
#   sudo apt update
#   sudo apt install build-essential binutils file
#   # 64-bit ARM (aarch64 / arm64), e.g. Jetson, RPi4/5 64-bit, most modern SBCs:
#   sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
#   # 32-bit ARM hard-float (armhf / armv7), e.g. RPi3 32-bit, older controllers:
#   sudo apt install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf
#
# Output: dist_<arch>/libGentoSDK.so and dist_<arch>/libGentoSDKPY.so
# (kept separate so x86 builds are NOT overwritten).
# Deploy by copying libGentoSDKPY.so next to the ARM-side GentoRobot.py.
#
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SDK_DIR="$SCRIPT_DIR/C_SDK"

CPP_FILES="
  ./L0Control/*.cpp
  ./FileClient/*.cpp
  ./L1Robot/*.cpp
  ./FXUtility/FXMath/*.cpp
  ./FXUtility/FXCfg/*.cpp
  ./Interf/*.cpp
  ./ToolDynaIdent/*.cpp
  ./Kinematics/*.cpp
  ./Kinematics/LunaBodyKinematics/*.cpp
  ./Kinematics/ArmKinematics/*.cpp
  ./Kinematics/KineCommon/*.cpp
  ./Kinematics/MotionPlanner/*.cpp
  ./Kinematics/SkyeBodyKinematics/*.cpp
"

INC_DIRS="
  -I./Common
  -I./FXUtility
  -I./FXUtility/FXMath
  -I./FXUtility/FXCfg
  -I./Interf
  -I./ToolDynaIdent
  -I./Kinematics
  -I./Kinematics/LunaBodyKinematics
  -I./Kinematics/ArmKinematics
  -I./Kinematics/KineCommon
  -I./Kinematics/MotionPlanner
  -I./Kinematics/SkyeBodyKinematics
  -I./FileClient
  -I./L0Control
  -I./L1Robot
"

usage() {
    echo "Usage: $0 [aarch64|armhf|armel|all]"
    echo "  aarch64  64-bit ARM (arm64)   [default]"
    echo "  armhf    32-bit ARM hard-float (armv7)"
    echo "  armel    32-bit ARM soft-float"
    echo "  all      build aarch64 + armhf"
}

# Set TRIPLET and required apt packages for the given arch.
config_arch() {
    case "$1" in
        aarch64|arm64)
            TRIPLET="aarch64-linux-gnu"
            PKGS="gcc-aarch64-linux-gnu g++-aarch64-linux-gnu"
            EXPECT_MACHINE="AArch64"
            ;;
        armhf|armv7)
            TRIPLET="arm-linux-gnueabihf"
            PKGS="gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf"
            EXPECT_MACHINE="ARM"
            ;;
        armel)
            TRIPLET="arm-linux-gnueabi"
            PKGS="gcc-arm-linux-gnueabi g++-arm-linux-gnueabi"
            EXPECT_MACHINE="ARM"
            ;;
        *)
            echo "[FAIL] Unsupported arch: $1"
            usage
            exit 1
            ;;
    esac
}

build_arch() {
    local ARCH="$1"
    config_arch "$ARCH"
    local CXX="${TRIPLET}-g++"
    local DIST="${SCRIPT_DIR}/dist_${ARCH}"

    echo "============================================"
    echo "Cross-compiling for ${ARCH} (${TRIPLET})"
    echo "============================================"

    if ! command -v "$CXX" >/dev/null 2>&1; then
        echo "[FAIL] Cross-compiler not found: $CXX"
        echo "       Install it first:"
        echo "       sudo apt install $PKGS"
        exit 1
    fi

    mkdir -p "$DIST"
    cd "$SDK_DIR"

    echo "[1/3] Building libGentoSDK.so (C/C++ link)"
    "$CXX" $CPP_FILES $INC_DIRS \
        -Wall -Wno-unused-result -std=gnu++17 -O2 -fPIC -shared \
        -o libGentoSDK.so \
        -Wl,--no-undefined \
        -lpthread -lrt -DCMPL_LIN
    echo "[OK] libGentoSDK.so built."

    echo "[2/3] Building libGentoSDKPY.so (Python ctypes)"
    "$CXX" $CPP_FILES $INC_DIRS \
        -Wall -Wno-unused-result -std=gnu++17 -O2 -fPIC -shared \
        -Wl,-soname,libGentoSDKPY.so \
        -Wl,--hash-style=both \
        -o libGentoSDKPY.so \
        -DL1_SDK_EXPORTS -DCMPL_LIN \
        -static-libgcc -static-libstdc++ \
        -Wl,--no-undefined \
        -lpthread -ldl -lm -lrt
    echo "[OK] libGentoSDKPY.so built."

    # Safety check: confirm the output really targets ARM (catches accidental host-g++ use).
    local MACHINE
    MACHINE=$(readelf -h libGentoSDKPY.so | awk '/Machine:/{print $2}')
    echo "[*]  ELF Machine: ${MACHINE}"
    if [ "$MACHINE" != "$EXPECT_MACHINE" ]; then
        echo "[FAIL] Built binary is '${MACHINE}', expected '${EXPECT_MACHINE}'."
        echo "       The cross-compiler may be misconfigured. Aborting before deploy."
        rm -f libGentoSDK.so libGentoSDKPY.so
        exit 1
    fi

    echo "[3/3] Copying SOs -> ${DIST}/"
    cp -v libGentoSDK.so   "$DIST/"
    cp -v libGentoSDKPY.so "$DIST/"
    rm -f libGentoSDK.so libGentoSDKPY.so

    echo "[OK] Done for ${ARCH}."
    echo "     Deploy: copy dist_${ARCH}/libGentoSDKPY.so next to the ARM-side GentoRobot.py"
    echo ""
}

ARCH="${1:-aarch64}"
case "$ARCH" in
    -h|--help)
        usage
        exit 0
        ;;
    all)
        build_arch aarch64
        build_arch armhf
        ;;
    *)
        build_arch "$ARCH"
        ;;
esac

echo "============================================"
echo "All done!"
echo "============================================"
