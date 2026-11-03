@echo off
REM Build test_link on Windows (MinGW / MSYS2)
setlocal

set SCRIPT_DIR=%~dp0
set SDK_DIR=%SCRIPT_DIR%..\C_SDK

set INC_DIRS= ^
  -I%SDK_DIR%\Common ^
  -I%SDK_DIR%\L0Control ^
  -I%SDK_DIR%\L1Robot ^
  -I%SDK_DIR%\Kinematics ^
  -I%SDK_DIR%\Kinematics\ArmKinematics ^
  -I%SDK_DIR%\Kinematics\BaseMath ^
  -I%SDK_DIR%\Kinematics\DynaIdent ^
  -I%SDK_DIR%\Kinematics\KineCommon ^
  -I%SDK_DIR%\Kinematics\MotionPlanner ^
  -I%SDK_DIR%\Kinematics\SkyeBodyKinematics ^
  -I%SDK_DIR%\FileClient

echo === Building test_link ===
g++ "%SCRIPT_DIR%test_link.cpp" ^
    %INC_DIRS% ^
    -L"%SDK_DIR%" -lGentoSDK ^
    -DCMPL_WIN ^
    -o "%SCRIPT_DIR%test_link.exe" ^
    -Wall -O2

if %ERRORLEVEL% EQU 0 (
    echo Build OK.  Binary: %SCRIPT_DIR%test_link.exe
    echo Run it directly: %SCRIPT_DIR%test_link.exe
) else (
    echo Build FAILED.
)
