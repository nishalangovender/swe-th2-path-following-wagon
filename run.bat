@echo off
REM Wagon Control System - Windows Setup and Run Script
REM This script creates a virtual environment, installs dependencies, and runs the wagon control system
REM Usage: run.bat [OPTIONS]
REM
REM Options:
REM   -v, --verbose      Enable verbose logging
REM   --no-plot          Skip all visualization
REM   --plot-only        Only visualize the most recent run (skip data collection)
REM   -h, --help         Show this help message

setlocal enabledelayedexpansion

REM Parse command-line arguments
set VERBOSE=
set PLOT_AFTER=true
set PLOT_ONLY=false
set SAVE_PLOTS=--save

:parse_args
if "%~1"=="" goto end_parse
if /i "%~1"=="-v" (
    set VERBOSE=--verbose
    shift
    goto parse_args
)
if /i "%~1"=="--verbose" (
    set VERBOSE=--verbose
    shift
    goto parse_args
)
if /i "%~1"=="--no-plot" (
    set PLOT_AFTER=false
    set SAVE_PLOTS=
    shift
    goto parse_args
)
if /i "%~1"=="--plot-only" (
    set PLOT_ONLY=true
    shift
    goto parse_args
)
if /i "%~1"=="-h" goto show_help
if /i "%~1"=="--help" goto show_help
echo Unknown option: %~1
echo Use --help for usage information
exit /b 1

:show_help
echo Usage: run.bat [OPTIONS]
echo.
echo Wagon Control System - Run data collection and visualization
echo.
echo Options:
echo   -v, --verbose      Enable verbose logging with timestamps
echo   --no-plot          Skip all visualization
echo   --plot-only        Only visualize the most recent run (skip collection)
echo   -h, --help         Show this help message
echo.
echo Examples:
echo   run.bat                    # Single run with live visualization
echo   run.bat --verbose          # Run with verbose logging
echo   run.bat --no-plot          # Run data collection only, no plots
echo   run.bat --plot-only        # Just visualize the last run
exit /b 0

:end_parse

echo ==============================
echo Wagon Control System Setup
echo ==============================

REM Check if Python is installed
python --version >nul 2>&1
if errorlevel 1 (
    echo Error: Python is not installed or not in PATH
    echo Please install Python 3.7+ from https://www.python.org/downloads/
    echo Make sure to check "Add Python to PATH" during installation
    exit /b 1
)

REM Get Python version
for /f "tokens=2" %%i in ('python --version 2^>^&1') do set PYTHON_VERSION=%%i
echo Found Python %PYTHON_VERSION%

REM Verify Python version (3.7+)
for /f "tokens=1,2 delims=." %%a in ("%PYTHON_VERSION%") do (
    set PYTHON_MAJOR=%%a
    set PYTHON_MINOR=%%b
)

if %PYTHON_MAJOR% LSS 3 (
    echo Error: Python 3.7+ required, found %PYTHON_VERSION%
    echo Please install Python 3.7 or higher
    exit /b 1
)

if %PYTHON_MAJOR% EQU 3 if %PYTHON_MINOR% LSS 7 (
    echo Error: Python 3.7+ required, found %PYTHON_VERSION%
    echo Please install Python 3.7 or higher
    exit /b 1
)

REM Check if venv module is available
python -c "import venv" >nul 2>&1
if errorlevel 1 (
    echo Error: Python venv module not found
    echo Please reinstall Python with the venv module included
    exit /b 1
)

REM Create virtual environment if it doesn't exist
if not exist "venv" (
    echo Creating virtual environment...
    python -m venv venv
    if errorlevel 1 (
        echo Error: Failed to create virtual environment
        exit /b 1
    )
    echo Created virtual environment
) else (
    echo Using existing virtual environment
)

REM Activate virtual environment
call venv\Scripts\activate.bat
if errorlevel 1 (
    echo Error: Failed to activate virtual environment
    exit /b 1
)

REM Upgrade pip
echo Upgrading pip...
python -m pip install --upgrade pip >nul 2>&1
if errorlevel 1 (
    echo Warning: Failed to upgrade pip, continuing anyway...
)

REM Install dependencies
echo Installing dependencies...
pip install -r requirements.txt >nul 2>&1
if errorlevel 1 (
    echo Error: Failed to install dependencies
    echo Try running: pip install -r requirements.txt
    exit /b 1
)
echo Installed dependencies
echo Setup completed!

echo.
echo =================================
echo Starting Wagon Control System
echo =================================

REM Run the wagon control system
if "%PLOT_ONLY%"=="true" (
    python -m wagon_control.plot_results
) else (
    python -m wagon_control.client %VERBOSE%

    REM Show plots after collection if requested
    if "%PLOT_AFTER%"=="true" (
        python -m wagon_control.plot_results %SAVE_PLOTS%
    )
)

echo.
echo Done!

REM Deactivate virtual environment
call venv\Scripts\deactivate.bat 2>nul

endlocal
