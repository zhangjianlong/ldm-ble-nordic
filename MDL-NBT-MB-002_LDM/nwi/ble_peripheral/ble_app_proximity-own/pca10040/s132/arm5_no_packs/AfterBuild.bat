::Build CMD: UV4 -r ble_app_proximity_pca10040_s132.uvprojx -t"nrf52832_custom"
set "BUILD_DIR=%cd%\_build"
set "AFETR_DIR=%cd%\..\..\..\..\..\..\..\BUILD"
set "OUT_HEX=application.hex"
set "BUILD_LOG=application.build_log.htm"

mkdir "%AFETR_DIR%"
copy "%BUILD_DIR%\%OUT_HEX%"   "%AFETR_DIR%\%OUT_HEX%"
copy "%BUILD_DIR%\%BUILD_LOG%" "%AFETR_DIR%\%BUILD_LOG%"
