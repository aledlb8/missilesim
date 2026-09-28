@echo off
rem MissileSim build helper.
rem
rem   build.bat            debug build    -> build\ninja-debug\bin\MissileSimOpenGL.exe
rem   build.bat release    release build  -> build\ninja-release\bin\MissileSimOpenGL.exe
rem   build.bat dist       shareable zip  -> build\dist\MissileSim-<version>-win64.zip
rem   build.bat clean      delete everything under build\
setlocal

set "MODE=%~1"
if "%MODE%"=="" set "MODE=debug"

if /I "%MODE%"=="clean" (
    if exist "%~dp0build" rd /s /q "%~dp0build"
    echo Removed build\
    exit /b 0
)

if /I "%MODE%"=="debug" (
    set "CONFIGURE_PRESET=ninja-debug"
    set "BUILD_PRESET=build-debug"
) else if /I "%MODE%"=="release" (
    set "CONFIGURE_PRESET=ninja-release"
    set "BUILD_PRESET=build-release"
) else if /I "%MODE%"=="dist" (
    rem handled by the workflow preset below
) else (
    echo Unknown mode "%MODE%". Usage: build.bat [debug^|release^|dist^|clean]
    exit /b 1
)

rem --- Locate Visual Studio (any edition, 2022 or newer) ----------------------
set "VS_INSTALL="
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if exist "%VSWHERE%" (
    rem "call" + %%VSWHERE%% defers expansion so the "(x86)" in the path
    rem cannot break the for /f command line.
    for /f "usebackq delims=" %%i in (`call "%%VSWHERE%%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VS_INSTALL=%%i"
)

rem --- Locate vcpkg: VCPKG_ROOT, then the copy bundled with Visual Studio -----
if not defined VCPKG_ROOT (
    if defined VS_INSTALL if exist "%VS_INSTALL%\VC\vcpkg\scripts\buildsystems\vcpkg.cmake" (
        set "VCPKG_ROOT=%VS_INSTALL%\VC\vcpkg"
    )
)
if not exist "%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake" (
    echo Unable to find vcpkg. Set VCPKG_ROOT or install the vcpkg component of Visual Studio.
    exit /b 1
)

rem --- Load the MSVC environment unless we are already in a developer shell ---
if not defined VSCMD_VER (
    if not defined VS_INSTALL (
        echo Unable to find Visual Studio with the C++ build tools.
        exit /b 1
    )
    call "%VS_INSTALL%\VC\Auxiliary\Build\vcvars64.bat" >nul
    if errorlevel 1 exit /b 1
)

pushd "%~dp0"

if /I "%MODE%"=="dist" (
    rem Configure + build + zip. The first run compiles the static vcpkg
    rem dependencies and takes a while; later runs reuse them.
    cmake --workflow --preset dist
    if errorlevel 1 (
        popd
        exit /b 1
    )
    echo.
    echo Shareable package:
    for %%f in ("build\dist\MissileSim-*-win64.zip") do echo   %%~ff
    popd
    exit /b 0
)

cmake --preset "%CONFIGURE_PRESET%"
if errorlevel 1 (
    popd
    exit /b 1
)
cmake --build --preset "%BUILD_PRESET%"
set "RESULT=%ERRORLEVEL%"
popd
exit /b %RESULT%
