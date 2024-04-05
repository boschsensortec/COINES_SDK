@echo off
setlocal enabledelayedexpansion

:: Get command line arguments
set "RemovePath=%~1"

:: RemovePathFromEnvironment
set "RegPath=HKLM\SYSTEM\CurrentControlSet\Control\Session Manager\Environment"

for /f "tokens=2*" %%a in ('reg query "%RegPath%" /v Path') do (
    set "OldPath=%%b"
)

if not defined OldPath set "OldPath="

set "NewPath="
for %%a in ("%OldPath:;=" "%") do (
    if not "%%~a"=="%RemovePath%" (
        if defined NewPath (
            set "NewPath=!NewPath!;"
        )
        set "NewPath=!NewPath!%%~a"
    )
)

reg add "%RegPath%" /v Path /t REG_EXPAND_SZ /d "%NewPath%" /f >nul

endlocal
