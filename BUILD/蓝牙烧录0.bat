@echo off

::set dir
set "FILE_DIR=%cd%"
::set hex file
set "HEX_FILE=output.hex"

echo ��ʼ��¼: %HEX_FILE%, �벻Ҫ�Ͽ�...
echo=

nrfjprog -f NRF52 --program "%FILE_DIR%\%HEX_FILE%" --chiperase --verify
IF ERRORLEVEL 1 goto ERROR
echo ��¼�ɹ�.
echo=
nrfjprog -f NRF52 --reset
IF ERRORLEVEL 1 goto ERROR
echo=
echo MCU�Ѹ�λ, ���鹦��.
echo=
goto END

:ERROR
echo=
echo ��������, ����!
echo=

:END
pause
