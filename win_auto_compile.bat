@echo off
setlocal enabledelayedexpansion

set SCRIPT_DIR=%~dp0
set SDK_DIR=%SCRIPT_DIR%C_SDK
set PYTHON_DIR=%SCRIPT_DIR%PYTHON_SDK
set C_DLL_DIR=%SCRIPT_DIR%C_EXAMPLE_USE_DLL_SO

set CPP_FILES=./L0Control/*.cpp ./FileClient/*.cpp ./L1Robot/*.cpp ./FXUtility/FXMath/*.cpp ./FXUtility/FXCfg/*.cpp ./Interf/*.cpp ./Kinematics/*.cpp ./Kinematics/ArmKinematics/*.cpp ./Kinematics/DynaIdent/*.cpp ./Kinematics/KineCommon/*.cpp ./Kinematics/MotionPlanner/*.cpp ./Kinematics/SkyeBodyKinematics/*.cpp

set INC_DIRS=-I./Common -I./FXUtility/ -I./FXUtility/FXMath -I./FXUtility/FXCfg -I./Interf -I./Kinematics -I./Kinematics/ArmKinematics -I./Kinematics/DynaIdent -I./Kinematics/KineCommon -I./Kinematics/MotionPlanner -I./Kinematics/SkyeBodyKinematics -I./FileClient -I./L0Control -I./L1Robot

cd /d "%SDK_DIR%"

echo ============================================
echo [1/3] Building libGentoSDKPY.dll (Python)
echo ============================================
g++ %CPP_FILES% %INC_DIRS% -Wall -O2 -shared -o libGentoSDKPY.dll -DL1_SDK_EXPORTS -DCMPL_WIN -static -static-libgcc -static-libstdc++ -lws2_32 -lwinmm
if !ERRORLEVEL! NEQ 0 (
    echo [FAIL] libGentoSDKPY.dll build failed!
    pause
    exit /b 1
)
echo [OK] libGentoSDKPY.dll built.

echo.
echo ============================================
echo [2/3] Building libGentoSDK.dll (C/C++ link)
echo ============================================
g++ %CPP_FILES% %INC_DIRS% -Wall -O2 -shared -o libGentoSDK.dll -DL1_SDK_EXPORTS -DCMPL_WIN -lws2_32 -lwinmm
if !ERRORLEVEL! NEQ 0 (
    echo [FAIL] libGentoSDK.dll build failed!
    pause
    exit /b 1
)
echo [OK] libGentoSDK.dll built.

cd /d "%SCRIPT_DIR%"

echo.
echo ============================================
echo [3/3] Copying DLLs to target folders...
echo ============================================

if not exist "%SDK_DIR%\libGentoSDKPY.dll" (
    echo [FAIL] Source file not found: %SDK_DIR%\libGentoSDKPY.dll
    pause
    exit /b 1
)
copy /Y "%SDK_DIR%\libGentoSDKPY.dll" "%PYTHON_DIR%\"
if !ERRORLEVEL! NEQ 0 (
    echo [FAIL] Copy libGentoSDKPY.dll failed!
    pause
    exit /b 1
)
echo [OK] libGentoSDKPY.dll copied to PYTHON_SDK\

if not exist "%SDK_DIR%\libGentoSDK.dll" (
    echo [FAIL] Source file not found: %SDK_DIR%\libGentoSDK.dll
    pause
    exit /b 1
)
copy /Y "%SDK_DIR%\libGentoSDK.dll" "%C_DLL_DIR%\"
if !ERRORLEVEL! NEQ 0 (
    echo [FAIL] Copy libGentoSDK.dll failed!
    pause
    exit /b 1
)
echo [OK] libGentoSDK.dll copied to C_EXAMPLE_USE_DLL_SO\

del /F /Q "%SDK_DIR%\libGentoSDK.dll" 2>nul
del /F /Q "%SDK_DIR%\libGentoSDKPY.dll" 2>nul
echo [OK] Delete C_SDK/libGentoSDK.dll  and  C_SDK/libGentoSDKPY.dll

echo.
echo ============================================
echo All done!
echo ============================================
pause
