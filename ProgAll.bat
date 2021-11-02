@echo off

set ID_COMPANY=EA

set ID_TOOLTYPE=81

::hex of 21060001
set ID_SN=014159A1
set OUTPUT_FOLDER=./PkgGenOutput/

if not exist %OUTPUT_FOLDER% (md "%OUTPUT_FOLDER%")

cd "%OUTPUT_FOLDER%"


set ALL_IN_ONE_HEX=NBT-MB-002_A20_B11production.hex



if not exist %ALL_IN_ONE_HEX% (
		echo "No %ALL_IN_ONE_HEX% Exists!"

	) else (


		nrfjprog -f NRF52 --eraseall
		nrfjprog -f NRF52 --program "%ALL_IN_ONE_HEX%"
		nrfjprog --memwr 0x10001080 --val 0x%ID_COMPANY%%ID_TOOLTYPE%
		nrfjprog --memwr 0x10001084 --val 0x%ID_SN%

		nrfjprog --reset
		
		echo "Program Complete!"

		
	)



