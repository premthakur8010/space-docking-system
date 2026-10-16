@echo off
echo ========================================================
echo Launching 4 Independent Space Docking UI Visualizers
echo (Cube, Sphere, Cone, Pyramid) @ 90 FPS via UDP Multicast
echo ========================================================

start "UI - Cube" "%~dp0build_win\docking_ui.exe" --shape cube --fps 90 --pos-x 50 --pos-y 50 --width 900 --height 600
start "UI - Sphere" "%~dp0build_win\docking_ui.exe" --shape sphere --fps 90 --pos-x 970 --pos-y 50 --width 900 --height 600
start "UI - Cone" "%~dp0build_win\docking_ui.exe" --shape cone --fps 90 --pos-x 50 --pos-y 670 --width 900 --height 600
start "UI - Pyramid" "%~dp0build_win\docking_ui.exe" --shape pyramid --fps 90 --pos-x 970 --pos-y 670 --width 900 --height 600

echo.
echo [OK] All 4 UI windows launched independently!
echo Closing any window will NOT affect any of the other windows or the physics engine.
