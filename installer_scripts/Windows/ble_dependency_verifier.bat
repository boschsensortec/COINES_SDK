@echo off
setlocal enabledelayedexpansion
set "dlls[0]=api-ms-win-crt-heap-l1-1-0.dll"
set "dlls[1]=api-ms-win-crt-runtime-l1-1-0.dll"
set "dlls[2]=api-ms-win-crt-math-l1-1-0.dll"
set "dlls[3]=api-ms-win-crt-stdio-l1-1-0.dll"
set "dlls[4]=api-ms-win-crt-locale-l1-1-0.dll"
set "dlls[5]=api-ms-win-crt-string-l1-1-0.dll"

set "missingDLLs="

REM Check if DLLs are missing
set counter=0
:check_dlls
if defined dlls[%counter%] (
    where /q !dlls[%counter%]!
    if errorlevel 1 (
        set "missingDLLs=!missingDLLs! !dlls[%counter%]!"
    )
    set /a counter+=1
    goto check_dlls
)

REM If missing DLLs are found, prompt for installation
if not "%missingDLLs%"=="" (
    echo Missing DLLs:
    for %%d in (%missingDLLs%) do (
        echo %%d
    )

    echo Exit code: 1
    exit /b 1  // Set exit code 1 indicating missing DLLs
)

echo All DLLs are present.
echo Exit code: 0
exit /b 0  // Set exit code 0 indicating all DLLs are present
