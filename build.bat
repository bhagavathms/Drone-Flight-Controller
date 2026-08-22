@echo off
REM =============================================================================
REM  build.bat -- Compile the Embedded C Flight Controller shared library
REM  Drone Flight Computer -- SITL Build Script (64-bit)
REM =============================================================================

echo.
echo [BUILD] Compiling flight_controller.c -^> flight_controller.dll (64-bit)
echo.

SET MSYS2_GCC=C:\msys64\usr\bin\bash.exe
SET BUILD_CMD=export PATH=/mingw64/bin:/usr/bin:$PATH; gcc -shared -o /c/Users/msbha/Desktop/DFC/flight_controller.dll /c/Users/msbha/Desktop/DFC/flight_controller.c -lm -Wall -O2

IF EXIST %MSYS2_GCC% (
    %MSYS2_GCC% -c "%BUILD_CMD%"
    IF %ERRORLEVEL% EQU 0 (
        echo.
        echo [BUILD] SUCCESS -- flight_controller.dll ^(64-bit^) is ready.
        echo [BUILD] Run:  python flight_sim.py
        echo.
    ) ELSE (
        echo [BUILD] FAILED -- Check error messages above.
        exit /b 1
    )
) ELSE (
    echo [BUILD] MSYS2 not found at C:\msys64
    echo [BUILD] Install via: winget install MSYS2.MSYS2
    echo [BUILD] Then run:    pacman -S mingw-w64-x86_64-gcc
    exit /b 1
)
