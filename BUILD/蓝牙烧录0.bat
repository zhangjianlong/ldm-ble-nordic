@echo off

::set dir
set "FILE_DIR=%cd%"
::set hex file
set "HEX_FILE=output.hex"

echo 开始烧录: %HEX_FILE%, 请不要断开...
echo=

nrfjprog -f NRF52 --program "%FILE_DIR%\%HEX_FILE%" --chiperase --verify
IF ERRORLEVEL 1 goto ERROR
echo 烧录成功.
echo=
nrfjprog -f NRF52 --reset
IF ERRORLEVEL 1 goto ERROR
echo=
echo MCU已复位, 请检查功能.
echo=
goto END

:ERROR
echo=
echo 发生错误, 请检查!
echo=

:END
pause
