@echo off
REM Run UI Client on Windows
if exist "build_win\docking_ui.exe" (
    "build_win\docking_ui.exe" %*
) else if exist "build\docking_ui.exe" (
    "build\docking_ui.exe" %*
) else (
    echo Please build the project first with cmake!
    pause
)
