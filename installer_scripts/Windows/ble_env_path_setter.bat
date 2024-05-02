@echo off
setlocal enabledelayedexpansion

:: Get command line arguments
set "NewPath=%~1"

:: UpdateEnvironmentPath
set "RegPath=HKLM\SYSTEM\CurrentControlSet\Control\Session Manager\Environment"
for /f "tokens=2*" %%a in ('reg query "%RegPath%" /v Path') do (
    set "OldPath=%%b"
)

if not defined OldPath set "OldPath="

:: Check if the path is already present in the environment variables
echo !OldPath! | find /i "%NewPath%" >nul
if errorlevel 1 (
    :: Append the new path to the existing environment variable value
    set "UpdatedPath=!OldPath!"
    if not defined OldPath (
        set "UpdatedPath=!NewPath!"
    ) else (
        set "UpdatedPath=!OldPath!;!NewPath!"
    )

    reg add "%RegPath%" /v Path /t REG_EXPAND_SZ /d "!UpdatedPath!" /f >nul
    if errorlevel 1 (
        echo Failed to update the environment variable.
    )
)

endlocal
