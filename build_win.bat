@echo off
setlocal
echo ========================================================
echo Building Space Docking System (Windows / MinGW)
echo ========================================================

where gcc >nul 2>&1
if %errorlevel% neq 0 (
    if exist "C:\msys64\mingw64\bin" (
        echo Adding C:\msys64\mingw64\bin to PATH...
        set "PATH=C:\msys64\mingw64\bin;%PATH%"
    )
)

cmake -B build_win -G "MinGW Makefiles" -DCMAKE_C_COMPILER=C:/msys64/mingw64/bin/gcc.exe -DCMAKE_CXX_COMPILER=C:/msys64/mingw64/bin/g++.exe -DCMAKE_MAKE_PROGRAM=C:/msys64/mingw64/bin/mingw32-make.exe
if %errorlevel% neq 0 (
    echo [ERROR] CMake configure failed!
    pause
    exit /b %errorlevel%
)

cmake --build build_win -j
if %errorlevel% neq 0 (
    echo [ERROR] Build failed!
    pause
    exit /b %errorlevel%
)

echo.
echo [SUCCESS] Build completed successfully!
echo Executables created in build_win\
echo   - build_win\physics_engine.exe
echo   - build_win\docking_ui.exe
