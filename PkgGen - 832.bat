set OUTPUT_FOLDER=./PkgGenOutput/

if not exist %OUTPUT_FOLDER% (md "%OUTPUT_FOLDER%")




::Base Path.
cd "%OUTPUT_FOLDER%"


set ALL_IN_ONE_HEX=NBT-MB-002_A20_B11production.hex

set DFU_PKG_APP=NBT-MB-002_A20_B11dfu.zip


set DFU_PKG_BOOT=NBT-MB-002_A20_B11bootdfu.zip
set BOOTLOADER_SETTINGS=bootloader_settings.hex
set timestamp=%date:~0,4%%date:~5,2%%date:~8,2%0%time:~1,1%%time:~3,2%%time:~6,2%

::App Hex Path. Based on OUTPUT_FOLDER.
set apphex=..\MDL-NBT-MB-002_LDM\nwi\ble_peripheral\ble_app_proximity-own\pca10040\s132\arm5_no_packs\_build\application.hex





::App Keil Project Path. Based on OUTPUT_FOLDER.
set appproject=..\MDL-NBT-MB-002_LDM\nwi\ble_peripheral\ble_app_proximity-own\pca10040\s132\arm5_no_packs\ble_app_proximity_pca10040_s132.uvprojx
::App Keil Configuration Name.
set apppconfiguration="nrf52832_custom"




set boothex=..\bootloader.hex

set privatekey=..\dfuprivate.key


set softdevice=..\softdevice.hex


move %DFU_PKG_APP% %DFU_PKG_APP%.%timestamp%
move %DFU_PKG_BOOT% %DFU_PKG_BOOT%.%timestamp%

move %BOOTLOADER_SETTINGS% %BOOTLOADER_SETTINGS%.%timestamp%


move %ALL_IN_ONE_HEX% %ALL_IN_ONE_HEX%.%timestamp%

::C:\Keil_v5\UV4\UV4.exe -r %appproject% -t%apppconfiguration%
..\nrfutil.exe pkg generate --hw-version 52 --application-version 1 --application %apphex% --sd-req 0x8C --key-file %privatekey% %DFU_PKG_APP%
..\nrfutil.exe pkg generate --hw-version 52 --bootloader-version 2 --bootloader %boothex% --sd-req 0x8C --key-file %privatekey% %DFU_PKG_BOOT%

..\nrfutil.exe settings generate --family NRF52 --application %apphex% --application-version 1 --bootloader-version 1 --bl-settings-version 1 %BOOTLOADER_SETTINGS%






mergehex.exe -m %softdevice% %boothex% %apphex% -o 3in1.hex&&mergehex.exe -m 3in1.hex %BOOTLOADER_SETTINGS% -o %ALL_IN_ONE_HEX%&&del 3in1.hex




