@echo off
REM install_deps.bat - Install Python dependencies for Webots controllers

echo Using Python at:
where python
echo.

python --version
if errorlevel 1 (
    echo ERROR: 'python' was not found on PATH.
    pause
    exit /b 1
)

echo.
echo Upgrading pip...
python -m pip install --upgrade pip

echo.
echo Installing dependencies from requirements.txt...
python -m pip install -r "%~dp0requirements.txt"
if errorlevel 1 (
    echo.
    echo ERROR: Failed to install dependencies.
    pause
    exit /b 1
)

echo.
echo Verifying installation...
python -c "import numpy; print('numpy', numpy.__version__, 'installed OK')"

echo.
echo Done.
pause