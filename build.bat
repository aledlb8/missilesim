@echo off
setlocal

set "CONFIGURE_PRESET=ninja-debug"
set "BUILD_PRESET=build-debug"
set "VCVARS64_BAT="

if /I "%~1"=="release" (
    set "CONFIGURE_PRESET=ninja-release"
    set "BUILD_PRESET=build-release"
)

if /I "%~1"=="clean" (
    if exist build (
        rd /s /q build
    )
    exit /b 0
)

if not defined VCPKG_ROOT (
    if exist "%ProgramFiles%\Microsoft Visual Studio\18\Community\VC\vcpkg\scripts\buildsystems\vcpkg.cmake" (
        set "VCPKG_ROOT=%ProgramFiles%\Microsoft Visual Studio\18\Community\VC\vcpkg"
    ) else if exist "%ProgramFiles%\Microsoft Visual Studio\2022\Community\VC\vcpkg\scripts\buildsystems\vcpkg.cmake" (
        set "VCPKG_ROOT=%ProgramFiles%\Microsoft Visual Studio\2022\Community\VC\vcpkg"
    ) else if exist "%USERPROFILE%\scoop\apps\vcpkg\current\scripts\buildsystems\vcpkg.cmake" (
        set "VCPKG_ROOT=%USERPROFILE%\scoop\apps\vcpkg\current"
    )
)

if not exist "%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake" (
    echo Unable to find vcpkg. Set VCPKG_ROOT or install vcpkg first.
    exit /b 1
)

if exist "%ProgramFiles%\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" (
    set "VCVARS64_BAT=%ProgramFiles%\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
) else if exist "%ProgramFiles%\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" (
    set "VCVARS64_BAT=%ProgramFiles%\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
)

if not defined VCVARS64_BAT (
    echo Unable to find Visual Studio build tools.
    exit /b 1
)

call "%VCVARS64_BAT%" >nul
if errorlevel 1 (
    exit /b 1
)

cmake --preset "%CONFIGURE_PRESET%"
if errorlevel 1 (
    exit /b 1
)

cmake --build --preset "%BUILD_PRESET%"
exit /b %ERRORLEVEL%
