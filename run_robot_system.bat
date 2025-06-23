@echo off
echo Robot System Launcher
echo ===================
echo.

REM Check if Python is installed
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: Python is not installed or not in PATH.
    echo Please install Python 3.6 or higher.
    pause
    exit /b 1
)

REM Run the launcher script
python run_robot_system.py %*

if %errorlevel% neq 0 (
    echo.
    echo An error occurred while running the launcher.
    pause
) 