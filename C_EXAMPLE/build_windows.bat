、@echo off
REM ==========================================================================
REM  Build one (or all) C_EXAMPLE example(s) -- Method 1:
REM  compile the example directly with all C_SDK source (no prebuilt library).
REM
REM  Usage:
REM    build_windows.bat                     -> builds example_basic_LinkSystem
REM    build_windows.bat example_basic_Position
REM    build_windows.bat example_basic_Position.cpp   (.cpp optional)
REM    build_windows.bat all                -> builds every example_*.cpp
REM
REM  Output: <name>.exe next to each source file. Run it yourself.
REM ==========================================================================
setlocal enabledelayedexpansion

set SCRIPT_DIR=%~dp0
set SDK_DIR=%SCRIPT_DIR%..\C_SDK

REM All C_SDK sources (relative to C_SDK; MinGW g++ expands the *.cpp globs).
set SDK_CPP=./FXUtility/FXMath/*.cpp ./FXUtility/FXCfg/*.cpp ./Interf/*.cpp ./Kinematics/*.cpp ./Kinematics/ArmKinematics/*.cpp ./Kinematics/DynaIdent/*.cpp ./Kinematics/KineCommon/*.cpp ./Kinematics/MotionPlanner/*.cpp ./Kinematics/SkyeBodyKinematics/*.cpp ./FileClient/*.cpp ./L0Control/*.cpp ./L1Robot/*.cpp

REM SDK include dirs (relative to C_SDK, same set as ..\win_auto_compile.bat).
set INC_DIRS=-I./Common -I./FXUtility -I./FXUtility/FXMath -I./FXUtility/FXCfg -I./Interf -I./Kinematics -I./Kinematics/ArmKinematics -I./Kinematics/DynaIdent -I./Kinematics/KineCommon -I./Kinematics/MotionPlanner -I./Kinematics/SkyeBodyKinematics -I./FileClient -I./L0Control -I./L1Robot

set TARGET=%~1
if "%TARGET%"=="" set TARGET=example_basic_LinkSystem
REM strip optional .cpp extension
if /I "!TARGET:~-4!"==".cpp" set TARGET=!TARGET:~0,-4!

if /I "%TARGET%"=="all" (
    echo Building every example_*.cpp in %SCRIPT_DIR%
    set FAIL=0
    for %%F in ("%SCRIPT_DIR%example_*.cpp") do (
        call :build_one "%%~nF"
        if !ERRORLEVEL! NEQ 0 set FAIL=1
    )
    echo.
    if "!FAIL!"=="1" ( echo [DONE] Some examples FAILED. ) else ( echo [DONE] All examples built OK. )
    pause
    exit /b
)

call :build_one "%TARGET%"
echo.
pause
exit /b

:build_one
set NAME=%~1
if not exist "%SCRIPT_DIR%!NAME!.cpp" (
    echo [FAIL] Not found: %SCRIPT_DIR%!NAME!.cpp
    exit /b 1
)
echo --------------------------------------------
echo Building !NAME!.cpp  ^(Method 1: compile with C_SDK source^)
echo --------------------------------------------
pushd "%SDK_DIR%"
g++ -w "%SCRIPT_DIR%!NAME!.cpp" %SDK_CPP% %INC_DIRS% -o "%SCRIPT_DIR%!NAME!.exe" -lws2_32 -lwinmm -DCMPL_WIN -DL1_SDK_EXPORTS
set RC=!ERRORLEVEL!
popd
if !RC! NEQ 0 (
    echo [FAIL] !NAME!.exe build failed ^(rc=!RC!^)
) else (
    echo [OK] !NAME!.exe    Run it: "%SCRIPT_DIR%!NAME!.exe"
)
exit /b !RC!
