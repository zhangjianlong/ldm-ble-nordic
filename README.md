# LDM-BLE-Nordic

彩屏测距仪（SBD tlm165s、DW03050系列）蓝牙模块软件。基于Nordic nRF52832。

## 发布配置


- 安装`nRF-Command-Line-Tools`。
- 将`nrfutil.exe`拷贝到`PkgGen - 832.bat`同一个目录。
- 运行`MDL-NBT-MB-002_LDM\nwi\ble_peripheral\ble_app_proximity-own\pca10040\s132\arm5_no_packs\ble_app_proximity_pca10040_s132.uvprojx`，选择`nrf52832_custom`配置，编译。
- 运行`PkgGen - 832.bat`。
- `PkgGenOutput`里的`production.hex`为生产烧录文件，`zip`文件为`dfu`文件。


## 烧录配置
- 安装`Segger JFlash`。
- 连接`JLINK`。
- 运行`ProgAll.bat`进行烧录。
