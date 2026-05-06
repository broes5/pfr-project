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
echo Installing numpy...
python -m pip install --upgrade pip
python -m pip install numpy

echo.
echo Verifying numpy installation...
python -c "import numpy; print('numpy', numpy.__version__, 'installed OK')"

echo.
pause