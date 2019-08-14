::2019.08.09. For Windows OS.
@echo off

echo "Start Generating BLE Programming Hex File."
echo "The Programming Hex File includes BOOTLOADER SETTING, BOOTLOADER, SOFTDEVICE, APPLICATION 4parts."



echo "Deleting old Files."
del /F bootloaer_setting.hex 3in1.hex output.hex 
echo "Generating BOOTLOADER SETTING(bootloader_setting.hex)"
nrfutil settings generate --family NRF52 --application application.hex --application-version 1 --bootloader-version 1 --bl-settings-version 1 bootloader_setting.hex
echo "Done!"


echo "Merging SOFTDEVICE(softdevice.hex), BOOTLOADER(bootloader.hex), APPLICATION(application.hex)"
mergehex -m .\resources\softdevice.hex bootloader.hex application.hex -o 3in1.hex
echo "Done!"


echo "Merging 3IN1(3in1.hex) and BOOTLOADER SETTING(bootloader_setting.hex)"
mergehex -m 3in1.hex bootloader_setting.hex -o .\output\output.hex
echo "Done!"
echo "All Done! Final Merged File is output.hex."

echo "Start Generating BLE Bootloader DFU zip File."
echo "The BLE Bootloader DFU zip File will get digital signed and all in the zip file."
echo "Deleting old Files."
del /F bootloader-dfu-packet-v2.zip
echo "Start signing bootloader(Version 2)."
nrfutil pkg generate --hw-version 52 --bootloader-version 2 --bootloader bootloader.hex --sd-req 0x8C --key-file .\resources\dfu-private.key .\output\bootloader-dfu-packet-v2.zip
echo "Done!"
echo "All Done! Final Signed File is bootloader-dfu-packet-v2.zip."


echo "Start Generating BLE Application DFU zip File."
echo "The BLE Application DFU zip File will get digital signed and all in the zip file."
echo "Deleting old Files."
del /F application-dfu-packet.zip
del /F 3in1.hex
del /F bootloader_setting.hex
echo "Start signing application."
nrfutil pkg generate --hw-version 52 --application-version 1 --application application.hex --sd-req 0x8C --key-file .\resources\dfu-private.key .\output\application-dfu-packet.zip
echo "Done!"
echo "All Done! Final Signed File is application-dfu-packet.zip."
