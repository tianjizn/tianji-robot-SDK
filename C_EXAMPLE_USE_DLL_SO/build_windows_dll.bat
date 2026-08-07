@echo off
REM ==========================================================================
REM  Build test code against the prebuilt libGentoSDK.dll -- Method 2.
REM  The DLL is produced by ..\win_auto_compile.bat and lives in this folder.
REM
REM  Usage:
REM    build_windows_dll.bat                       -> builds local test_link.cpp
REM    build_windows_dll.bat example_basic_Position
REM    build_windows_dll.bat example_basic_Position.cpp   (.cpp optional)
REM    build_windows_dll.bat all                   -> builds every ..\C_EXAMPLE\example_*.cpp
REM
REM  Example sources are read from ..\C_EXAMPLE\. Output <name>.exe lands here.
REM  Run the executable yourself.
REM ==========================================================================
setlocal enabledelayedexpansion

set SCRIPT_DIR=%~dp0
set SDK_DIR=%SCRIPT_DIR%..\C_SDK
set EX_DIR=%SCRIPT_DIR%..\C_EXAMPLE

REM SDK include dirs (relative to C_SDK; same set as ..\win_auto_compile.bat).
set INC_DIRS=-I./Common -I./FXUtility -I./FXUtility/FXMath -I./FXUtility/FXCfg -I./Interf -I./Kinematics -I./Kinematics/ArmKinematics -I./Kinematics/DynaIdent -I./Kinematics/KineCommon -I./Kinematics/MotionPlanner -I./Kinematics/SkyeBodyKinematics -I./FileClient -I./L0Control -I./L1Robot

echo ============================================
echo Checking compiled library
echo ============================================
if not exist "%SCRIPT_DIR%libGentoSDK.dll" (
    echo [FAIL] libGentoSDK.dll not found in %SCRIPT_DIR%
    echo        Run ..\win_auto_compile.bat first to compile the SDK.
    pause
    exit /b 1
)
echo [OK] libGentoSDK.dll found.

set TARGET=%~1
if "%TARGET%"=="" set TARGET=test_link
if /I "!TARGET:~-4!"==".cpp" set TARGET=!TARGET:~0,-4!

if /I "%TARGET%"=="all" (
    echo Building every example_*.cpp from %EX_DIR% ^(Method 2: link prebuilt DLL^)
    set FAIL=0
    for %%F in ("%EX_DIR%\example_*.cpp") do (
        call :build_one "%%~nF" "%EX_DIR%\%%~nF.cpp"
        if !ERRORLEVEL! NEQ 0 set FAIL=1
    )
    echo.
    if "!FAIL!"=="1" ( echo [DONE] Some examples FAILED. ) else ( echo [DONE] All examples built OK. )
    pause
    exit /b
)

REM resolve source: this folder first, then ..\C_EXAMPLE
set SRC=%SCRIPT_DIR%!TARGET!.cpp
if not exist "!SRC!" set SRC=%EX_DIR%\!TARGET!.cpp
call :build_one "!TARGET!" "!SRC!"
echo.
pause
exit /b

:build_one
set NAME=%~1
set SRC=%~2
if not exist "!SRC!" (
    echo [FAIL] Not found: !SRC!
    exit /b 1
)
echo --------------------------------------------
echo Building !NAME!.cpp  ^(Method 2: link prebuilt DLL^)
echo --------------------------------------------
REM -include FXCmplOpt.h : pull winsock/atomic in up front (MinGW/GCC quirk under -O2).
REM Link the DLL directly (no import lib) so ld resolves the __imp_* import thunks.
pushd "%SDK_DIR%"
g++ -include FXCmplOpt.h "!SRC!" %INC_DIRS% "%SCRIPT_DIR%libGentoSDK.dll" -DCMPL_WIN -Wall -O2 -o "%SCRIPT_DIR%!NAME!.exe"
set RC=!ERRORLEVEL!
popd
if !RC! NEQ 0 (
    echo [FAIL] !NAME!.exe build failed ^(rc=!RC!^)
) else (
    echo [OK] !NAME!.exe    Run: "%SCRIPT_DIR%!NAME!.exe"
)
exit /b !RC!
