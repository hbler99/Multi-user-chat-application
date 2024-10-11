# 基于 Micro-ROS 的多人语音聊天例程

- [English Version](./README.md)
- 例程难度：![alt text](./docs/_static/level_regular.png "中级")


## 例程简介

本例程演示了 Micro-ROS 在 ADF 框架下订阅和发布音频流的过程。

本例程的管道如下图：

```
[codec_chip] ---> i2s_stream_reader ---> [Micro-ROS publisher]
                                                |
                                                v
                                            [PC ROS2]
                                                ^
                                                |                
                                        [Micro-ROS subscribtion] ---> i2s_stream_writer ---> [codec_chip]
```


## 环境配置

### 硬件要求

本例程支持的开发板在 `$ADF_PATH/examples/README_CN.md` 文档中[例程与乐鑫音频开发板的兼容性表格](../../README_CN.md#例程与乐鑫音频开发板的兼容性)中有标注，表格中标有绿色复选框的开发板均可运行本例程。请记住，如下面的 [配置](#配置) 一节所述，可以在 `menuconfig` 中选择开发板。


## 编译和下载

### IDF 默认分支

本例程支持 IDF release/v5.2 ，例程默认使用 ADF 的內建分支 `$ADF_PATH/esp-idf`。

### 配置
本例程需要 PC 端安装 ROS2，建议安装 [Humble](https://docs.ros.org/en/humble/Installation.html) 版本。

如果您是第一次使用 `Micro-ROS`，请先按照 https://github.com/micro-ROS/micro_ros_espidf_component?tab=readme-ov-file#example 进行配置。

本例程默认选择的开发板是 `ESP32-S3-Korvo-2`，如果需要在其他的开发板上运行此例程，则需要在 menuconfig 中选择开发板的配置，例如选择 `ESP32-Lyrat V4.3`。

```
menuconfig > Audio HAL > ESP32-Lyrat V4.3
```

本例程需要额外安装 `Micro-ROS` 的依赖库，已放在 `components` 中，如果发现本地文件没有安装该库， 请使用下述命令：

```
git submodule update --init --recursive
```

本例需要连接 Wi-Fi 网络用于 `Micro-ROS` 通信，通过运行 `menuconfig > micro-ROS Settings` 来配置 Wi-Fi 信息。

```
 menuconfig > micro-ROS Settings > WiFi Configuration > WiFi SSID
 menuconfig > micro-ROS Settings > WiFi Configuration > WiFi Password
```

本例程需要配置 `Micro-ROS` 代理信息（IP 地址和端口号），通过运行 `menuconfig > micro-ROS Settings` 填写 `micro-ROS Agent IP` 和 `micro-ROS Agent Port`。

```
menuconfig > micro-ROS Settings > (PC ROS2 IP) micro-ROS Agent IP 
menuconfig > micro-ROS Settings > (PC ROS2 Port) micro-ROS Agent Port
```

建议开启 `PSARAM` 实现稳定运行。
```
menuconfig > Component config > ESP PSRAM > Support for external, SPI-connected RAM
``` 

如果您用的开发板是 `8 线 SPI RAM`，请配置
```
menuconfig > Component config > ESP PSRAM > Support for external, SPI-connected RAM > SPI RAM config > Mode (QUAD/OCT) of SPI RAM chip in use (Octal Mode PSRAM)
```

建议配置 `FreeRTOS` 时钟频率为 1000 Hz
```
menuconfig > Component config > FreeRTOS > Kernel > (1000) configTICK_RATE_HZ
```

### 运行
### 编译和下载 
请先编译版本并烧录到开发板上，然后运行 monitor 工具来查看串口输出（替换 PORT 为端口名称）：

```
idf.py -p PORT flash monitor
```

退出调试界面使用 ``Ctrl-]``

有关配置和使用 ESP-IDF 生成项目的完整步骤，请参阅 [《ESP-IDF 编程指南》](https://docs.espressif.com/projects/esp-idf/zh_CN/release-v5.2/esp32/index.html)。


## 如何使用例程

### 功能和用法

- 例程需要先运行 Micros-ROS 代理，并且和开发板连接在同一个 Wi-Fi 网络中，于 PC 端使用以下docker命令启动 Micros-ROS 代理：

```
# UDPv4 micro-ROS Agent
sudo docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port [PC ROS2 Port] -v6
```

- 例程提供 python3.x 脚本 [Audio_com_sub.py](./tools/Audio_com_sub.py), 用于接收 ROS2 话题消息，并实时播放AMR音频。使用脚本请先安装 pyaudio, subprocess, struct 库：
```
pip install pyaudio subprocess struct
```

- 运行脚本，使用请注意更改 [ros2 topic](./tools/Audio_com_sub.py#L90) 名称：
```
python3 Audio_com_sub.py
```

- 本例程需要至少两个板子实现音频通信，同时发布音频数据发布到 ROS2 话题和订阅话题中音频数据并播放。分别设定 **1 号板**和 **2 号板**。
  - 1 号板：
    - 发布 [ros2 topic](./main/Micro_ROS_audio_com.c#L419) 名称为 “Esp32_audio_data_1”
    -  订阅[ros2 topic](./main/Micro_ROS_audio_com.c#L426) 名称为 “Esp32_audio_data_2”
  - 2 号板：
    - 发布 [ros2 topic](./main/Micro_ROS_audio_com.c#L419) 名称为 “Esp32_audio_data_2”
    - 订阅[ros2 topic](./main/Micro_ROS_audio_com.c#L426) 名称为 “Esp32_audio_data_1”
  
- 例程开始运行后，打印如下：
```c
rst:0x1 (POWERON),boot:0x8 (SPI_FAST_FLASH_BOOT)
SPIWP:0xee
mode:DIO, clock div:1
load:0x3fce3810,len:0x178c
load:0x403c9700,len:0x4
load:0x403c9704,len:0xcbc
load:0x403cc700,len:0x2d98
entry 0x403c9914
I (27) boot: ESP-IDF v5.2-dirty 2nd stage bootloader
I (27) boot: compile time Sep 19 2024 20:12:32
I (27) boot: Multicore bootloader
I (30) boot: chip revision: v0.2
I (34) boot.esp32s3: Boot SPI Speed : 80MHz
I (39) boot.esp32s3: SPI Mode       : DIO
I (44) boot.esp32s3: SPI Flash Size : 4MB
I (48) boot: Enabling RNG early entropy source...
I (54) boot: Partition Table:
I (57) boot: ## Label            Usage          Type ST Offset   Length
I (65) boot:  0 nvs              WiFi data        01 02 00009000 00006000
I (72) boot:  1 phy_init         RF data          01 01 0000f000 00001000
I (79) boot:  2 factory          factory app      00 00 00010000 00300000
I (87) boot: End of partition table
I (91) esp_image: segment 0: paddr=00010020 vaddr=3c0c0020 size=3ec80h (257152) map
I (146) esp_image: segment 1: paddr=0004eca8 vaddr=3fc99b00 size=01370h (  4976) load
I (147) esp_image: segment 2: paddr=00050020 vaddr=42000020 size=b84e8h (754920) map
I (287) esp_image: segment 3: paddr=00108510 vaddr=3fc9ae70 size=04e8ch ( 20108) load
I (291) esp_image: segment 4: paddr=0010d3a4 vaddr=40374000 size=15a6ch ( 88684) load
I (321) boot: Loaded app from partition at offset 0x10000
I (321) boot: Disabling RNG early entropy source...
I (333) cpu_start: Multicore app
I (333) octal_psram: vendor id    : 0x0d (AP)
I (333) octal_psram: dev id       : 0x02 (generation 3)
I (336) octal_psram: density      : 0x03 (64 Mbit)
I (342) octal_psram: good-die     : 0x01 (Pass)
I (347) octal_psram: Latency      : 0x01 (Fixed)
I (352) octal_psram: VCC          : 0x01 (3V)
I (357) octal_psram: SRF          : 0x01 (Fast Refresh)
I (363) octal_psram: BurstType    : 0x01 (Hybrid Wrap)
I (369) octal_psram: BurstLen     : 0x01 (32 Byte)
I (375) octal_psram: Readlatency  : 0x02 (10 cycles@Fixed)
I (381) octal_psram: DriveStrength: 0x00 (1/1)
I (386) esp_psram: Found 8MB PSRAM device
I (390) esp_psram: Speed: 40MHz
I (1125) esp_psram: SPI SRAM memory test OK
I (1134) cpu_start: Pro cpu start user code
I (1134) cpu_start: cpu freq: 240000000 Hz
I (1135) cpu_start: Application information:
I (1138) cpu_start: Project name:     Micro_ROS_audio_com
I (1144) cpu_start: App version:      218d5df-dirty
I (1149) cpu_start: Compile time:     Oct 11 2024 14:09:26
I (1156) cpu_start: ELF file SHA256:  a173635ef...
I (1161) cpu_start: ESP-IDF:          v5.2-dirty
I (1166) cpu_start: Min chip rev:     v0.0
I (1171) cpu_start: Max chip rev:     v0.99 
I (1176) cpu_start: Chip rev:         v0.2
I (1181) heap_init: Initializing. RAM available for dynamic allocation:
I (1188) heap_init: At 3FCA69D8 len 00042D38 (267 KiB): RAM
I (1194) heap_init: At 3FCE9710 len 00005724 (21 KiB): RAM
I (1201) heap_init: At 3FCF0000 len 00008000 (32 KiB): DRAM
I (1207) heap_init: At 600FE010 len 00001FD8 (7 KiB): RTCRAM
I (1213) esp_psram: Adding pool of 8192K of PSRAM memory to heap allocator
I (1221) spi_flash: detected chip: gd
I (1225) spi_flash: flash io: dio
W (1229) spi_flash: Detected size(16384k) larger than the size in the binary image header(4096k). Using the size in the binary image header.
W (1242) i2c: This driver is an old driver, please migrate your application code to adapt `driver/i2c_master.h`
W (1253) ADC: legacy driver is deprecated, please migrate to `esp_adc/adc_oneshot.h`
I (1262) sleep: Configure to isolate all GPIO pins in sleep state
I (1268) sleep: Enable automatic switching of GPIO sleep configuration
I (1276) coexist: coex firmware version: 77cd7f8
I (1281) coexist: coexist rom version e7ae62f
I (1287) main_task: Started on CPU0
I (1291) esp_psram: Reserving pool of 32K of internal memory for DMA/internal allocations
I (1300) main_task: Calling app_main()
I (1304) Audio_com: [ 1 ] Initialize peripherals
I (1310) AUDIO_THREAD: The button_task task allocate stack on external memory
I (1318) AUDIO_THREAD: The esp_periph task allocate stack on internal memory
I (1325) Audio_com: [ 2 ] Start codec chip
I (1330) DRV8311: es8311_codec_init
I (1339) DRV8311: ES8311 in Slave mode
I (1351) gpio: GPIO[48]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0 
W (1361) I2C_BUS: I2C bus has been already created, [port:0]
I (1366) ES7210: ES7210 in Slave mode
I (1376) ES7210: Enable ES7210_INPUT_MIC1
I (1380) ES7210: Enable ES7210_INPUT_MIC2
I (1383) ES7210: Enable ES7210_INPUT_MIC3
W (1386) ES7210: Enable TDM mode. ES7210_SDP_INTERFACE2_REG12: 2
I (1392) ES7210: config fmt 60
I (1394) AUDIO_HAL: Codec mode is 3, Ctrl:1
I (1403) Audio_com: [ 3 ] Create and start input key service
I (1403) AUDIO_THREAD: The input_key_service task allocate stack on internal memory
I (1411) Audio_com: [ 4 ] Create audio pipeline for playback
I (1417) Audio_com: [4.1] Create i2s stream to write data to codec chip and read data from codec chip
I (1431) Audio_com: [4.2] Register all elements to audio pipeline
I (1434) Audio_com: [4.3] Link it together [Micro-ros]-->i2s_stream_writer-->amr_decoder-->[codec_chip]     [codec_chip]-->i2s_stream_reader-->amrnb_encoder-->[Micro-ros]
I (1450) AUDIO_PIPELINE: link el->rb, el:0x3c101d50, tag:i2s_reader, rb:0x3c102b14
I (1458) AUDIO_PIPELINE: link el->rb, el:0x3c1022b4, tag:amr_decoder, rb:0x3c104b5c
I (1467) Audio_com: [ 5 ] Set up  event listener
I (1472) Audio_com: [ 6 ] Start audio_pipeline
W (1477) Audio_com: [ 7 ] Press the [Play] keys to communicate
I (1506) wifi_station_netif: ESP_WIFI_MODE_STA
I (1507) pp: pp rom version: e7ae62f
I (1508) net80211: net80211 rom version: e7ae62f
I (1511) wifi:wifi driver task: 3fcd595c, prio:23, stack:6656, core=0
I (1525) wifi:wifi firmware version: cc1dd81
I (1525) wifi:wifi certification version: v7.0
I (1526) wifi:config NVS flash: enabled
I (1527) wifi:config nano formating: disabled
I (1532) wifi:Init data frame dynamic rx buffer num: 32
I (1536) wifi:Init static rx mgmt buffer num: 5
I (1541) wifi:Init management short buffer num: 32
I (1545) wifi:Init static tx buffer num: 16
I (1549) wifi:Init tx cache buffer num: 32
I (1553) wifi:Init static tx FG buffer num: 2
I (1557) wifi:Init static rx buffer size: 1600
I (1561) wifi:Init static rx buffer num: 10
I (1565) wifi:Init dynamic rx buffer num: 32
I (1570) wifi_init: rx ba win: 6
I (1573) wifi_init: tcpip mbox: 32
I (1577) wifi_init: udp mbox: 6
I (1581) wifi_init: tcp mbox: 6
I (1585) wifi_init: tcp tx win: 5760
I (1589) wifi_init: tcp rx win: 5760
I (1593) wifi_init: tcp mss: 1440
I (1597) wifi_init: WiFi IRAM OP enabled
I (1602) wifi_init: WiFi RX IRAM OP enabled
I (1608) phy_init: phy_version 620,ec7ec30,Sep  5 2023,13:49:13
I (1700) wifi:mode : sta (74:4d:bd:9d:98:84)
I (1701) wifi:enable tsf
I (1703) wifi_station_netif: wifi_init_sta finished.
I (1739) wifi:new:<1,0>, old:<1,0>, ap:<255,255>, sta:<1,0>, prof:1
I (1740) wifi:state: init -> auth (b0)
I (1809) wifi:state: auth -> assoc (0)
I (1846) wifi:state: assoc -> run (10)
I (1847) wifi:connected with AudioTest, aid = 3, channel 1, BW20, bssid = 24:cf:24:5a:09:e2
I (1847) wifi:security: Open Auth, phy: bgn, rssi: -12
I (1854) wifi:pm start, type: 1

I (1855) wifi:dp: 1, bi: 102400, li: 3, scale listen interval from 307200 us to 307200 us
I (1863) wifi:set rx beacon pti, rx_bcn_pti: 14, bcn_timeout: 25000, mt_pti: 14, mt_time: 10000
I (1872) wifi:APs beacon interval = 102400 us, DTIM period = 1
W (1882) wifi:[ADDBA]rx delba, code:39, delete tid:0
W (1883) wifi:[ADDBA]rx delba, code:39, delete tid:0
W (1886) wifi:[ADDBA]rx delba, code:39, delete tid:6
I (1892) wifi:<ba-add>idx:0 (ifx:0, 24:cf:24:5a:09:e2), tid:6, ssn:0, winSize:64
I (2879) esp_netif_handlers: sta ip: 192.168.31.140, mask: 255.255.255.0, gw: 192.168.31.1
I (2880) wifi_station_netif: got ip:192.168.31.140
I (2884) wifi_station_netif: connected to ap SSID:AudioTest password:
I (2916) wifi:<ba-add>idx:1 (ifx:0, 24:cf:24:5a:09:e2), tid:0, ssn:0, winSize:64
I (3107) AUDIO_THREAD: The i2s_reader task allocate stack on internal memory
I (3108) AUDIO_ELEMENT: [i2s_reader-0x3c101d50] Element task created
I (3112) AUDIO_THREAD: The amr_encoder task allocate stack on external memory
I (3120) AUDIO_ELEMENT: [amr_encoder-0x3c10215c] Element task created
I (3127) AUDIO_PIPELINE: Func:audio_pipeline_run, Line:359, MEM Total:8373684 Bytes, Inter:48279 Bytes, Dram:48279 Bytes, Dram largest free:21504Bytes

I (3141) AUDIO_ELEMENT: [i2s_reader] AEL_MSG_CMD_RESUME,state:1
I (3148) AUDIO_ELEMENT: [amr_encoder] AEL_MSG_CMD_RESUME,state:1
I (3154) AMRWB_ENCODER: amrwb open
I (3155) AUDIO_PIPELINE: Pipeline started
I (3163) AUDIO_THREAD: The i2s_writer task allocate stack on internal memory
I (3171) AUDIO_ELEMENT: [i2s_writer-0x3c102570] Element task created
I (3178) AUDIO_THREAD: The amr_decoder task allocate stack on external memory
I (3186) AUDIO_ELEMENT: [amr_decoder-0x3c1022b4] Element task created
I (3193) AUDIO_PIPELINE: Func:audio_pipeline_run, Line:359, MEM Total:8357760 Bytes, Inter:44003 Bytes, Dram:44003 Bytes, Dram largest free:17408Bytes

I (3207) AUDIO_ELEMENT: [i2s_writer] AEL_MSG_CMD_RESUME,state:1
I (3214) AUDIO_ELEMENT: [amr_decoder] AEL_MSG_CMD_RESUME,state:1
< (3221) Audio_com: Write AMRWB_HEADER_INFO: #!AMR-
I (3226) Audio_com: Write AMRWB_HEADER_INFO: WB
<R-
I (3232) CODEC_ELEMENT_HELPER: The element is 0x3c1022b4. The reserve data 2 is 0x0.
I (3241) AMR_DECODER: a new song playing
I (3246) AMR_DECODER: current audio is amrwb
I (3318) AUDIO_PIPELINE: Pipeline started
```

- 使用音量减小按键 [Vol-] 降低音量, 音量增大按键 [Vol+] 增大音量.

```c
I (621051) Audio_com: [ * ] [Vol+] input key event
I (621053) Audio_com: [ * ] Volume set to 80 %
I (622091) Audio_com: [ * ] [Vol-] input key event
I (622092) Audio_com: [ * ] Volume set to 70 %

```


- 1 号板和 2 号板按下 [Play] 按键，开始对话功能，打印如下：

```c
I (713796) Audio_com: [ * ] [Play] input key event
W (713796) Audio_com: START communicating
```

- 1 号板和 2 号板按下 [Stop] 按键，停止对话功能，打印如下：

```c
I (713796) Audio_com: [ * ] [Stop] input key event
W (713796) Audio_com: STOP communicating
```

### 日志输出
以下为本例程的完整日志。

```c
rst:0x1 (POWERON),boot:0x8 (SPI_FAST_FLASH_BOOT)
SPIWP:0xee
mode:DIO, clock div:1
load:0x3fce3810,len:0x178c
load:0x403c9700,len:0x4
load:0x403c9704,len:0xcbc
load:0x403cc700,len:0x2d98
entry 0x403c9914
I (27) boot: ESP-IDF v5.2-dirty 2nd stage bootloader
I (27) boot: compile time Sep 19 2024 20:12:32
I (27) boot: Multicore bootloader
I (30) boot: chip revision: v0.2
I (34) boot.esp32s3: Boot SPI Speed : 80MHz
I (39) boot.esp32s3: SPI Mode       : DIO
I (44) boot.esp32s3: SPI Flash Size : 4MB
I (48) boot: Enabling RNG early entropy source...
I (54) boot: Partition Table:
I (57) boot: ## Label            Usage          Type ST Offset   Length
I (65) boot:  0 nvs              WiFi data        01 02 00009000 00006000
I (72) boot:  1 phy_init         RF data          01 01 0000f000 00001000
I (79) boot:  2 factory          factory app      00 00 00010000 00300000
I (87) boot: End of partition table
I (91) esp_image: segment 0: paddr=00010020 vaddr=3c0c0020 size=3ec80h (257152) map
I (146) esp_image: segment 1: paddr=0004eca8 vaddr=3fc99b00 size=01370h (  4976) load
I (147) esp_image: segment 2: paddr=00050020 vaddr=42000020 size=b84e8h (754920) map
I (287) esp_image: segment 3: paddr=00108510 vaddr=3fc9ae70 size=04e8ch ( 20108) load
I (291) esp_image: segment 4: paddr=0010d3a4 vaddr=40374000 size=15a6ch ( 88684) load
I (321) boot: Loaded app from partition at offset 0x10000
I (321) boot: Disabling RNG early entropy source...
I (333) cpu_start: Multicore app
I (333) octal_psram: vendor id    : 0x0d (AP)
I (333) octal_psram: dev id       : 0x02 (generation 3)
I (336) octal_psram: density      : 0x03 (64 Mbit)
I (342) octal_psram: good-die     : 0x01 (Pass)
I (347) octal_psram: Latency      : 0x01 (Fixed)
I (352) octal_psram: VCC          : 0x01 (3V)
I (357) octal_psram: SRF          : 0x01 (Fast Refresh)
I (363) octal_psram: BurstType    : 0x01 (Hybrid Wrap)
I (369) octal_psram: BurstLen     : 0x01 (32 Byte)
I (375) octal_psram: Readlatency  : 0x02 (10 cycles@Fixed)
I (381) octal_psram: DriveStrength: 0x00 (1/1)
I (386) esp_psram: Found 8MB PSRAM device
I (390) esp_psram: Speed: 40MHz
I (1125) esp_psram: SPI SRAM memory test OK
I (1134) cpu_start: Pro cpu start user code
I (1134) cpu_start: cpu freq: 240000000 Hz
I (1135) cpu_start: Application information:
I (1138) cpu_start: Project name:     Micro_ROS_audio_com
I (1144) cpu_start: App version:      218d5df-dirty
I (1149) cpu_start: Compile time:     Oct 11 2024 14:09:26
I (1156) cpu_start: ELF file SHA256:  a173635ef...
I (1161) cpu_start: ESP-IDF:          v5.2-dirty
I (1166) cpu_start: Min chip rev:     v0.0
I (1171) cpu_start: Max chip rev:     v0.99 
I (1176) cpu_start: Chip rev:         v0.2
I (1181) heap_init: Initializing. RAM available for dynamic allocation:
I (1188) heap_init: At 3FCA69D8 len 00042D38 (267 KiB): RAM
I (1194) heap_init: At 3FCE9710 len 00005724 (21 KiB): RAM
I (1201) heap_init: At 3FCF0000 len 00008000 (32 KiB): DRAM
I (1207) heap_init: At 600FE010 len 00001FD8 (7 KiB): RTCRAM
I (1213) esp_psram: Adding pool of 8192K of PSRAM memory to heap allocator
I (1221) spi_flash: detected chip: gd
I (1225) spi_flash: flash io: dio
W (1229) spi_flash: Detected size(16384k) larger than the size in the binary image header(4096k). Using the size in the binary image header.
W (1242) i2c: This driver is an old driver, please migrate your application code to adapt `driver/i2c_master.h`
W (1253) ADC: legacy driver is deprecated, please migrate to `esp_adc/adc_oneshot.h`
I (1262) sleep: Configure to isolate all GPIO pins in sleep state
I (1268) sleep: Enable automatic switching of GPIO sleep configuration
I (1276) coexist: coex firmware version: 77cd7f8
I (1281) coexist: coexist rom version e7ae62f
I (1287) main_task: Started on CPU0
I (1291) esp_psram: Reserving pool of 32K of internal memory for DMA/internal allocations
I (1300) main_task: Calling app_main()
I (1304) Audio_com: [ 1 ] Initialize peripherals
I (1310) AUDIO_THREAD: The button_task task allocate stack on external memory
I (1318) AUDIO_THREAD: The esp_periph task allocate stack on internal memory
I (1325) Audio_com: [ 2 ] Start codec chip
I (1330) DRV8311: es8311_codec_init
I (1339) DRV8311: ES8311 in Slave mode
I (1351) gpio: GPIO[48]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0 
W (1361) I2C_BUS: I2C bus has been already created, [port:0]
I (1366) ES7210: ES7210 in Slave mode
I (1376) ES7210: Enable ES7210_INPUT_MIC1
I (1380) ES7210: Enable ES7210_INPUT_MIC2
I (1383) ES7210: Enable ES7210_INPUT_MIC3
W (1386) ES7210: Enable TDM mode. ES7210_SDP_INTERFACE2_REG12: 2
I (1392) ES7210: config fmt 60
I (1394) AUDIO_HAL: Codec mode is 3, Ctrl:1
I (1403) Audio_com: [ 3 ] Create and start input key service
I (1403) AUDIO_THREAD: The input_key_service task allocate stack on internal memory
I (1411) Audio_com: [ 4 ] Create audio pipeline for playback
I (1417) Audio_com: [4.1] Create i2s stream to write data to codec chip and read data from codec chip
I (1431) Audio_com: [4.2] Register all elements to audio pipeline
I (1434) Audio_com: [4.3] Link it together [Micro-ros]-->i2s_stream_writer-->amr_decoder-->[codec_chip]     [codec_chip]-->i2s_stream_reader-->amrnb_encoder-->[Micro-ros]
I (1450) AUDIO_PIPELINE: link el->rb, el:0x3c101d50, tag:i2s_reader, rb:0x3c102b14
I (1458) AUDIO_PIPELINE: link el->rb, el:0x3c1022b4, tag:amr_decoder, rb:0x3c104b5c
I (1467) Audio_com: [ 5 ] Set up  event listener
I (1472) Audio_com: [ 6 ] Start audio_pipeline
W (1477) Audio_com: [ 7 ] Press the [Play] keys to communicate
I (1506) wifi_station_netif: ESP_WIFI_MODE_STA
I (1507) pp: pp rom version: e7ae62f
I (1508) net80211: net80211 rom version: e7ae62f
I (1511) wifi:wifi driver task: 3fcd595c, prio:23, stack:6656, core=0
I (1525) wifi:wifi firmware version: cc1dd81
I (1525) wifi:wifi certification version: v7.0
I (1526) wifi:config NVS flash: enabled
I (1527) wifi:config nano formating: disabled
I (1532) wifi:Init data frame dynamic rx buffer num: 32
I (1536) wifi:Init static rx mgmt buffer num: 5
I (1541) wifi:Init management short buffer num: 32
I (1545) wifi:Init static tx buffer num: 16
I (1549) wifi:Init tx cache buffer num: 32
I (1553) wifi:Init static tx FG buffer num: 2
I (1557) wifi:Init static rx buffer size: 1600
I (1561) wifi:Init static rx buffer num: 10
I (1565) wifi:Init dynamic rx buffer num: 32
I (1570) wifi_init: rx ba win: 6
I (1573) wifi_init: tcpip mbox: 32
I (1577) wifi_init: udp mbox: 6
I (1581) wifi_init: tcp mbox: 6
I (1585) wifi_init: tcp tx win: 5760
I (1589) wifi_init: tcp rx win: 5760
I (1593) wifi_init: tcp mss: 1440
I (1597) wifi_init: WiFi IRAM OP enabled
I (1602) wifi_init: WiFi RX IRAM OP enabled
I (1608) phy_init: phy_version 620,ec7ec30,Sep  5 2023,13:49:13
I (1700) wifi:mode : sta (74:4d:bd:9d:98:84)
I (1701) wifi:enable tsf
I (1703) wifi_station_netif: wifi_init_sta finished.
I (1739) wifi:new:<1,0>, old:<1,0>, ap:<255,255>, sta:<1,0>, prof:1
I (1740) wifi:state: init -> auth (b0)
I (1809) wifi:state: auth -> assoc (0)
I (1846) wifi:state: assoc -> run (10)
I (1847) wifi:connected with AudioTest, aid = 3, channel 1, BW20, bssid = 24:cf:24:5a:09:e2
I (1847) wifi:security: Open Auth, phy: bgn, rssi: -12
I (1854) wifi:pm start, type: 1

I (1855) wifi:dp: 1, bi: 102400, li: 3, scale listen interval from 307200 us to 307200 us
I (1863) wifi:set rx beacon pti, rx_bcn_pti: 14, bcn_timeout: 25000, mt_pti: 14, mt_time: 10000
I (1872) wifi:APs beacon interval = 102400 us, DTIM period = 1
W (1882) wifi:[ADDBA]rx delba, code:39, delete tid:0
W (1883) wifi:[ADDBA]rx delba, code:39, delete tid:0
W (1886) wifi:[ADDBA]rx delba, code:39, delete tid:6
I (1892) wifi:<ba-add>idx:0 (ifx:0, 24:cf:24:5a:09:e2), tid:6, ssn:0, winSize:64
I (2879) esp_netif_handlers: sta ip: 192.168.31.140, mask: 255.255.255.0, gw: 192.168.31.1
I (2880) wifi_station_netif: got ip:192.168.31.140
I (2884) wifi_station_netif: connected to ap SSID:AudioTest password:
I (2916) wifi:<ba-add>idx:1 (ifx:0, 24:cf:24:5a:09:e2), tid:0, ssn:0, winSize:64
I (3107) AUDIO_THREAD: The i2s_reader task allocate stack on internal memory
I (3108) AUDIO_ELEMENT: [i2s_reader-0x3c101d50] Element task created
I (3112) AUDIO_THREAD: The amr_encoder task allocate stack on external memory
I (3120) AUDIO_ELEMENT: [amr_encoder-0x3c10215c] Element task created
I (3127) AUDIO_PIPELINE: Func:audio_pipeline_run, Line:359, MEM Total:8373684 Bytes, Inter:48279 Bytes, Dram:48279 Bytes, Dram largest free:21504Bytes

I (3141) AUDIO_ELEMENT: [i2s_reader] AEL_MSG_CMD_RESUME,state:1
I (3148) AUDIO_ELEMENT: [amr_encoder] AEL_MSG_CMD_RESUME,state:1
I (3154) AMRWB_ENCODER: amrwb open
I (3155) AUDIO_PIPELINE: Pipeline started
I (3163) AUDIO_THREAD: The i2s_writer task allocate stack on internal memory
I (3171) AUDIO_ELEMENT: [i2s_writer-0x3c102570] Element task created
I (3178) AUDIO_THREAD: The amr_decoder task allocate stack on external memory
I (3186) AUDIO_ELEMENT: [amr_decoder-0x3c1022b4] Element task created
I (3193) AUDIO_PIPELINE: Func:audio_pipeline_run, Line:359, MEM Total:8357760 Bytes, Inter:44003 Bytes, Dram:44003 Bytes, Dram largest free:17408Bytes

I (3207) AUDIO_ELEMENT: [i2s_writer] AEL_MSG_CMD_RESUME,state:1
I (3214) AUDIO_ELEMENT: [amr_decoder] AEL_MSG_CMD_RESUME,state:1
< (3221) Audio_com: Write AMRWB_HEADER_INFO: #!AMR-
I (3226) Audio_com: Write AMRWB_HEADER_INFO: WB
<R-
I (3232) CODEC_ELEMENT_HELPER: The element is 0x3c1022b4. The reserve data 2 is 0x0.
I (3241) AMR_DECODER: a new song playing
I (3246) AMR_DECODER: current audio is amrwb
I (3318) AUDIO_PIPELINE: Pipeline started
I (621051) Audio_com: [ * ] [Vol+] input key event
I (621053) Audio_com: [ * ] Volume set to 80 %
I (622091) Audio_com: [ * ] [Vol-] input key event
I (622092) Audio_com: [ * ] Volume set to 70 %
I (713796) Audio_com: [ * ] [Play] input key event
W (713796) Audio_com: START communicating
I (713796) Audio_com: [ * ] [Stop] input key event
W (713796) Audio_com: STOP communicating
```

## 故障排除

- 目前阶段，仅支持双人聊天对话，增加板子可以订阅话题播放；如果需要多人聊天对话，请自行增加混音模块实现。


## 技术支持
请按照下面的链接获取技术支持：

- 技术支持参见 [esp32.com](https://esp32.com/viewforum.php?f=20) 论坛
- 故障和新功能需求，请创建 [GitHub issue](https://github.com/espressif/esp-adf/issues)

我们会尽快回复。
