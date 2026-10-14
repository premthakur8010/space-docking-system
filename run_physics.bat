@echo off
REM Run Physics Engine Server on Windows
if exist "build_win\physics_engine.exe" (
    "build_win\physics_engine.exe" %*
) else if exist "build\physics_engine.exe" (
    "build\physics_engine.exe" %*
) else (
    echo [ERROR] physics_engine.exe not found!
    echo Please build the project first: cmake -B build_win -G "MinGW Makefiles" ^&^& cmake --build build_win
    pause
)
