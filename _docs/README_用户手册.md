# AquaMonitor 水质监测仪用户手册

> 文档版本：1.0
> 更新日期：2026-02-22
> 适用固件版本：AquaMonitor 主分支
> 项目仓库：https://github.com/your-username/AquaMonitor

---

## 项目简介

AquaMonitor 是一款基于 **ESP32 微控制器**的高精度水质多参数监测仪，可同时测量 **电导率、温度、pH 值** 等关键水质指标。系统采用模块化硬件设计，通过模拟多路复用器分时接入不同传感器，利用 **ADI AD5940/AD5941 电化学模拟前端** 实现阻抗谱测量与直流电流检测，配合 **TI ADS124S08 精密 Σ-Δ ADC** 完成铂热电阻温度测量。

### 主要功能特性

- **多参数测量**：
  - **电导率**：采用两端交流阻抗法（50 kHz 正弦激励），测量范围 0~200 mS/cm（取决于电极常数）
  - **温度**：PT1000 铂热电阻，比率法测量，范围 -200°C ~ +850°C（实际常用 0~50°C）
  - **pH 值**：三电极电化学传感器，直流安培法（Nernst 方程换算待实现）
  - **余氯**：预留通道，电化学法（暂未实现）

- **硬件特性**：
  - 主控：ESP32（双核 240 MHz，4 MB Flash）
  - 模拟前端：AD5940/AD5941（阻抗测量 + 电化学检测）
  - 温度 ADC：ADS124S08（24 位 Σ-Δ，PGA，IDAC 激励）
  - 通道切换：TMUX 模拟多路复用器（2×4 路差分 + 1×8 路单端）
  - 通信接口：USB UART（115200 bps）+ 蓝牙 SPP（Classic）

- **软件特性**：
  - 固件基于 Arduino 框架（PlatformIO 构建）
  - 单线程协作式状态机，无 RTOS 开销
  - 全 ASCII 文本命令协议，便于上位机集成
  - NVS Flash 存储校准系数，掉电不丢失
  - 支持三点温度多项式校准、电导率电极常数标定、pH 零点/增益校准

### 应用场景

- 实验室水质分析
- 水产养殖水质监控
- 环境监测站
- 工业过程水质控制
- 教学与科研实验平台

### 系统架构概览

```
传感器阵列
├── 电导率电极（阻抗法）───┐
├── pH 电极（电流法）───────┼──→ MUX-A（2位地址）──→ AD5940
├── 余氯电极（预留）───────┘
├── PT1000 温度传感器 ──────→ ADS124S08
└── ISFET 阵列（1~8路）──→ MUX-B（3位地址）──→ 前端电路

主控：ESP32
├── SPI 总线（共享，8/4 MHz）
├── UART（115200，命令/数据）
├── GPIO（MUX 地址线控制）
└── NVS Flash（校准存储）
```

---

## 硬件要求

### 核心硬件清单

| 部件 | 型号/规格 | 数量 | 备注 |
|------|-----------|------|------|
| **主控制器** | ESP32（esp32dev） | 1 | 双核 240MHz，4MB Flash |
| **阻抗/电化学 AFE** | AD5940 或 AD5941 | 1 | 阻抗测量 + pH 电流检测 |
| **温度 ADC** | ADS124S08 | 1 | 24 位 Σ-Δ，内置 PGA、IDAC |
| **模拟多路复用器** | TMUX1134（类） | 1 | 2-bit，4 路差分（传感通道） |
| **模拟多路复用器** | TMUX1108（类） | 1 | 3-bit，8 路单端（ISFET 通道） |
| **温度传感器** | PT1000 铂热电阻 | 1 | IEC 60751，三线制 |
| **电导率电极** | 两端电极，K 常数已知 | 1 | 需标定电极常数 |
| **pH 电极** | 三电极电化学传感器 | 1 | 工作电极 + 参比电极 + 对电极 |
| **参比电阻** | 3300 Ω，0.1% 精度 | 1 | 温度测量比率法用 |
| **校准电阻** | 1000 Ω，0.1% 精度 | 1 | AD5940 RTIA 校准用 |

### 电源要求

- **供电电压**：5 VDC（通过 USB 或外部电源）
- **工作电流**：
  - 静态：~80 mA（ESP32 + 外设待机）
  - 测量峰值：~120 mA（AD5940 32MHz 模式）
- **建议电源**：USB 5V/1A 或 DC 5V/2A 稳压电源

### 接口与连接器

| 接口 | 类型 | 引脚定义 | 用途 |
|------|------|---------|------|
| **传感器接口** | 端子排/防水接头 | 参见原理图 | 连接电导率、pH、温度传感器 |
| **USB** | Micro-USB | D+/D- | 供电 + UART 通信 |
| **扩展接口** | GPIO 排针 | 见 GPIO 分配表 | 外部扩展、调试 |
| **蓝牙天线** | PCB 天线 | 内置 | 2.4 GHz Classic Bluetooth |

### 最小系统配置

1. **ESP32 开发板**（需引出足够 GPIO）
2. **AD5941 评估板** 或自制 AFE 模块
3. **ADS124S08 模块** 或集成电路
4. **传感器阵列**（至少包含电导率电极和 PT1000）
5. **USB 转 UART 调试器**（如果开发板无内置）

> **注意**：完整硬件设计需参考原理图与 PCB 布局文件，本文档仅提供功能说明。

---

## 开发环境搭建

### 工具链安装

#### 1. 安装 PlatformIO（推荐）

PlatformIO 是跨平台的嵌入式开发平台，支持 ESP32 Arduino 框架。

- **VS Code 扩展**：
  1. 安装 [Visual Studio Code](https://code.visualstudio.com/)
  2. 在扩展商店搜索 "PlatformIO IDE" 并安装
  3. 重启 VS Code，PlatformIO 图标将出现在侧边栏

- **命令行安装**（可选）：
  ```bash
  # 安装 PlatformIO Core
  pip install platformio
  ```

#### 2. 安装 ESP32 Arduino 框架

PlatformIO 会自动下载所需框架，如需手动配置：

1. 在 `platformio.ini` 中指定：
   ```ini
   platform = espressif32
   board = esp32dev
   framework = arduino
   ```

2. 首次构建时 PlatformIO 将下载工具链（约 300 MB）

#### 3. 安装串口调试工具

- **Windows**：安装 [CP210x USB 驱动](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)
- **Linux/macOS**：通常自带驱动，可能需要权限配置

### 依赖库

项目依赖以下 Arduino 库（已在 `platformio.ini` 中声明）：

| 库名 | 版本 | 用途 |
|------|------|------|
| `ArduinoJson` | ^6.21.0 | JSON 解析（预留） |
| `BluetoothSerial` | ESP32 内置 | 蓝牙 SPP 通信 |
| `Preferences` | ESP32 内置 | NVS Flash 存储 |

**无需手动安装**，PlatformIO 会自动下载。

### 项目结构说明

```
AquaMonitor/
├── platformio.ini          # PlatformIO 构建配置
├── CMakeLists.txt          # CMake 配置（备用）
├── src/                    # 源代码
│   ├── main.cpp            # 主程序入口、状态机
│   ├── ad5940.c            # ADI 官方驱动（纯 C）
│   ├── ad5941_board_glue.cpp  # AD5941 ESP32 适配
│   ├── Conductivity_service.cpp  # 电导率服务
│   ├── ph_service.cpp      # pH 测量服务
│   ├── temp_service.cpp    # 温度服务
│   ├── ads124s08_drv.cpp   # ADS124S08 驱动
│   ├── mux_iface.cpp       # 多路复用器控制
│   └── ...（其他模块）
├── include/                # 头文件
│   ├── ad5940.h            # AD5940 驱动头文件
│   ├── Conductivity_service.h
│   ├── ph_service.h
│   ├── temp_service.h
│   └── ...（其他头文件）
└── _docs/                  # 文档目录
    ├── 软件架构说明.md
    ├── 测量原理说明_论文版.md
    └── README_用户手册.md（本文档）
```

### 编译与构建

1. **打开项目**：
   ```bash
   cd /path/to/AquaMonitor
   code .   # 或用 VS Code 打开文件夹
   ```

2. **构建固件**：
   - VS Code：点击底部状态栏 "→ PlatformIO: Build"
   - 命令行：`pio run`

3. **编译输出**：
   - `.pio/build/esp32dev/firmware.bin`：可烧录固件
   - `.pio/build/esp32dev/firmware.elf`：调试符号文件

### 开发注意事项

1. **SPI 总线共享**：AD5940 与 ADS124S08 共享 VSPI 总线，通过不同 CS 引脚区分
2. **GPIO 冲突**：避免使用已分配的 GPIO（见 GPIO 分配表）
3. **中断处理**：本项目采用轮询而非硬件中断，避免在中断服务例程中执行耗时操作
4. **内存使用**：ESP32 可用 RAM 约 320KB，固件当前占用约 40KB，留有充足余量

---

## 快速开始

### 硬件连接

#### 1. 传感器连接

| 传感器 | 接口引脚 | 连接说明 |
|--------|---------|---------|
| **电导率电极** | MUX-A CH1（S1A/S1B） | 两端电极，无极性要求 |
| **pH 电极** | MUX-A CH3（S3A/S3B） | 三电极：WE→S3A，RE/CE→S3B |
| **余氯电极** | MUX-A CH2（S2A/S2B） | 预留，暂未实现 |
| **PT1000** | ADS124S08 AIN1/AIN2 | 三线制：高端→AIN1，低端→AIN2，屏蔽→AGND |
| **ISFET 阵列** | MUX-B CH1~CH8 | 单端输出，共地连接 |

> **注意**：实际接线请参考硬件原理图，确保电源、地线正确连接。

#### 2. 电源与通信连接

- **USB 供电**：通过 Micro-USB 连接电脑或 5V 适配器
- **串口调试**：USB 同时提供 UART 通信（115200 bps）
- **外部供电**：如需独立供电，确保 5V 稳定，GND 共地

#### 3. 上电检查

1. 连接 USB，观察 ESP32 电源指示灯（常亮）
2. AD5940/ADS124S08 上电指示灯（如有）应亮起
3. 串口调试工具应收到启动日志：
   ```
   AquaMonitor System Boot...
   Device ID: 0
   SPI Bus initialized.
   MUX channels set to default.
   ```

### 编译固件

#### PlatformIO 快速编译

```bash
# 进入项目目录
cd AquaMonitor

# 编译项目
pio run

# 编译并上传到已连接的 ESP32
pio run --target upload

# 监视串口输出
pio device monitor
```

#### 编译参数说明

- **板型**：`esp32dev`（在 `platformio.ini` 中定义）
- **框架**：Arduino
- **优化等级**：-Os（尺寸优化）
- **Flash 模式**：`qio` 80MHz

### 烧录方法

#### 1. USB 自动烧录（推荐）

PlatformIO 支持一键烧录：

- **VS Code**：点击状态栏 "→ PlatformIO: Upload"
- **命令行**：`pio run --target upload`

#### 2. 手动烧录（esptool.py）

```bash
# 安装 esptool
pip install esptool

# 擦除 Flash
esptool.py --chip esp32 --port COM3 erase_flash

# 烧录固件
esptool.py --chip esp32 --port COM3 --baud 921600 \
  write_flash 0x1000 .pio/build/esp32dev/firmware.bin
```

#### 3. OTA 升级（预留）

系统预留 OTA 升级接口，待实现。

### 首次运行测试

烧录完成后，打开串口监视器（115200 bps），发送测试命令：

1. **读取温度**：
   ```
   temp read
   ```
   预期响应：`Water Temp: 25.123456`

2. **初始化电导率测量**：
   ```
   cond init
   cond read
   ```
   预期响应：`Conductivity: 1413.0000`（示例值）

3. **初始化 pH 测量**：
   ```
   ph init
   ph read
   ```
   预期响应：电流值（μA）

如果收到正常响应，说明硬件连接与固件运行正常。

---

## 配置说明

### 关键参数配置

系统配置参数存储在 ESP32 NVS Flash 中，上电自动加载。可通过串口命令查看和修改。

#### 电导率测量参数

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| 激励频率 | 50 kHz | 1 kHz ~ 200 kHz | 正弦波频率，影响测量精度 |
| 激励幅度 | 600 mVpp | 40 mVpp ~ 1600 mVpp | 自动档位选择 |
| RTIA 电阻 | 1 kΩ | 200 Ω ~ 160 kΩ | 电流检测量程 |
| 电极常数 K | 1.0 cm⁻¹ | >0 | 需用标准液标定 |
| 输出数据率 | 20 Hz | 0.5 Hz ~ 200 Hz | WUPT 定时器周期 |

**配置命令**：
- `cond init`：使用当前参数初始化
- `cond cal`：用标准液校准电极常数 K

#### 温度测量参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| 参比电阻 Rref | 3300 Ω | 比率法基准，硬件固定 |
| PGA 增益 | 2× | ADC 内部增益 |
| IDAC 电流 | 500 μA | PT1000 激励电流 |
| 校准系数 a,b,c | 0,1,0 | 三点多项式系数 |

**校准命令**：
- `temp cal 25` / `temp cal 35` / `temp cal 50`：记录校准点
- `temp save`：计算并保存校准系数

#### pH 测量参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| 偏置电压 Vbias | ~1.135 V | 工作电极直流偏置 |
| 零点电压 Vzero | ~0.898 V | 参比/对电极参考 |
| LPTIA RTIA | 200 Ω | 电流检测电阻 |
| 零点偏移码 | 32768 | ADC 中点，需校准 |
| RTIA 增益值 | 1000.0 Ω | 计算用，需校准 |

**校准命令**：
- `ph cal offset`：零点校准（pH=7 缓冲液）
- `ph cal gain <R>`：增益校准（接已知电阻 R）

### 通信接口配置

#### 1. 串口通信（默认启用）

- **波特率**：115200
- **数据位**：8
- **停止位**：1
- **校验位**：无
- **流控制**：无

**帧格式**：
```
命令：<ASCII 字符串>\n
响应：<多行文本>\n
```

#### 2. 蓝牙 SPP（需代码启用）

固件已实现蓝牙服务，但 `main.cpp` 中默认未启用。如需启用：

1. 在 `main.cpp` 中添加：
   ```cpp
   BTService_Init("AquaMonitor");
   ```
2. 在 `loop()` 中添加：
   ```cpp
   BTService_ReadCommand(cmdBuffer);
   ```

**蓝牙配对**：
- 设备名称：`AquaMonitor`
- 配对码：`1234`（默认）
- 服务：SPP（串口协议）

#### 3. 自定义通信协议

可通过修改 `host_link.cpp` 实现自定义协议（当前为空实现）。

### 设备 ID 设置

首次上电或执行 `factory reset` 后，设备 ID 为 0。设置设备 ID：

```
id <编号>
```

示例：`id 1` → 设备 ID 保存为 1，重启后生效。

> **注意**：设备 ID 仅可设置一次，如需修改需执行 `factory reset`。

---

## 数据输出格式说明

### 命令响应格式

所有响应均为 **ASCII 纯文本**，以换行符 `\n` 分隔。

#### 1. 单行响应

```
<标签>: <数值> <单位>
```

示例：
```
Water Temp: 25.123456
Resistance: 1097.32 Ohm
Conductivity: 1413.0000
```

#### 2. 多行响应

以 `>>>` 开始，`<<<` 结束的多行数据块：

```
>>> Sweep Started <<<
Freq(Hz)    Zreal(Ohm)    Zimag(Ohm)
1000.0      1234.56        -78.90
2000.0      1198.76        -82.34
...
>>> Sweep Completed! <<<
```

#### 3. 状态/信息响应

```
[CMD] <操作描述>
<结果信息>
```

示例：
```
[CMD] Initializing pH Measurement...
pH Service Init OK!
```

#### 4. 错误响应

直接输出错误描述：

```
Not in Conductivity Mode
Conductivity Service haven't been initialized
>>> Error: Signal too low (0.0000V). Is resistor connected? <<<
```

### 测量数据格式

#### 温度数据

```
Water Temp: <温度值>      # 单位 °C，6位小数
```

示例：`Water Temp: 25.123456`

**原始电阻值**（调试用）：
```
Resistance: <电阻值> Ohm   # 单位 Ω，2位小数
```

#### 电导率数据

```
Conductivity: <电导率值>   # 单位 μS/cm，4位小数
```

示例：`Conductivity: 1413.0000`

**阻抗谱数据**（扫频模式）：

```
Freq(Hz)    Zreal(Ohm)    Zimag(Ohm)
<频率>      <实部>         <虚部>
...
```

- 频率：Hz，1位小数
- 实部/虚部：Ω，2位小数

#### pH 数据（当前为电流值）

```
Current: <电流值> uA       # 单位 μA，4位小数
```

示例：`Current: -123.4567 uA`

> **注意**：当前固件输出的是电极电流，pH 值换算待实现。

### 校准数据输出

#### 温度校准系数

```
Calibration DONE! a=0.000123, b=0.999876, c=-0.123456
```

#### 电导率电极常数

```
>>> Calibration Successful! <<<
K_Cell=1.023
```

#### pH 校准参数

```
>>> Offset Calibrated! New Zero Code: 0x8000 (32768) <<<
>>> Gain Calibrated! New RTIA: 985.23 Ohm <<<
```

### 数据记录建议

上位机程序可解析以下模式：

1. **定时采集**：周期性发送 `temp read`、`cond read`、`ph read`
2. **批量导出**：`cond sweep` 输出 CSV 格式阻抗谱
3. **校准日志**：保存校准系数及时间戳

---

## 故障排除

### 常见问题与解决方案

#### 1. 上电无响应

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| 电源指示灯不亮 | 供电异常 | 检查 USB 线、电源电压 |
| 无串口输出 | USB 驱动问题 | 安装 CP210x 驱动，更换 USB 端口 |
| 串口输出乱码 | 波特率不匹配 | 确保串口工具设置为 115200 bps |
| 持续复位 | 电源纹波大 | 增加电源滤波电容，使用稳压电源 |

#### 2. 传感器测量异常

**电导率测量值为 0 或异常大**：
- 检查电极连接是否牢固
- 执行 `cond cal` 重新校准电极常数
- 确认溶液电导率在量程内（1 kΩ RTIA 对应 ~1 mS/cm 量程）

**温度测量值不稳定**：
- PT1000 接线松动（三线制需补偿导线电阻）
- 参比电阻 Rref（3300 Ω）精度不足
- 执行温度三点校准 `temp cal 25/35/50` → `temp save`

**pH 电流值为 0**：
- 检查 pH 电极连接（三电极：WE、RE、CE）
- 执行 `ph cal offset` 零点校准
- 确认缓冲液 pH 值正确

#### 3. 通信问题

**串口命令无响应**：
- 检查命令格式（以 `\n` 结尾）
- 确认设备未处于持续测量状态（发送其他命令退出）
- 重启设备

**蓝牙无法连接**：
- 确认 `bt_service.cpp` 已集成到 `main.cpp`
- 检查 ESP32 蓝牙天线（PCB 天线需净空区）
- 重新配对（删除已配对设备）

#### 4. 校准失败

**温度校准点保存失败**：
- 三点温度差异需 >0.1°C
- 每个校准点采集时传感器需稳定 30 秒
- 标准温度计精度需高于 0.1°C

**电导率校准无效**：
- 标准液电导率值需准确（建议使用 NIST 可追溯标准液）
- 校准时电极需完全浸入，无气泡
- 温度补偿：标准液电导率值为 25°C 下值

**pH 校准信号太低**：
- 校准电阻连接不良
- 检查 `ph cal gain <R>` 命令中电阻值是否正确
- 确认 AD5940 LPTIA 电路工作正常

### 错误代码与含义

| 错误信息 | 含义 | 处理建议 |
|---------|------|---------|
| `Not in Conductivity Mode` | 未初始化电导率模式 | 先发送 `cond init` |
| `Not in pH Mode` | 未初始化 pH 模式 | 先发送 `ph init` |
| `... Service haven't been initialized` | 服务未初始化 | 执行对应初始化命令 |
| `!!! ADS124S08 configuration FAILED! Halting. !!!` | 温度 ADC 配置失败 | 检查 SPI 连接，重启设备 |
| `>>> Error: Signal too low ... <<<` | 校准信号太小 | 检查传感器/电阻连接 |

### 调试技巧

1. **查看原始数据**：
   - `temp resistance`：查看 PT1000 原始电阻
   - pH 电流值：反映电极响应强度

2. **SPI 总线调试**：
   - 逻辑分析仪抓取 SPI 波形（SCLK、MOSI、MISO、CS）
   - 确认 AD5940/ADS124S08 寄存器读写正常

3. **电源监测**：
   - 测量 3.3V、5V 电源纹波（<50 mVpp）
   - 检查地线回路，避免数字噪声耦合到模拟部分

4. **固件日志**：
   - 在 `main.cpp` 中添加 `Serial.printf` 调试输出
   - 使用 `printf` 格式化复杂数据结构

### 恢复出厂设置

如果参数配置混乱，可执行：

```
factory reset
```

此命令将：
- 清除所有校准系数
- 重置设备 ID 为 0
- 恢复默认测量参数

**注意**：出厂设置后需重新校准所有传感器。

---

## 二次开发指南

### 固件架构概述

固件采用分层架构，便于功能扩展与修改：

```
应用层（main.cpp）
├── 状态机管理
├── 命令解析
└── 服务协调
    │
服务层（*_service.cpp）
├── 电导率服务（Conductivity_service）
├── pH 服务（ph_service）
├── 温度服务（temp_service）
└── 游离氯服务（freecl_service，空）
    │
驱动层（*_drv.cpp, ad5940.c）
├── AD5940 驱动（ADI 官方）
├── ADS124S08 驱动
└── 板级胶合层（*_board_glue）
    │
硬件抽象层（HAL）
├── SPI 抽象（spi_hal）
└── MUX 控制（mux_iface）
```

### 添加新测量参数

以添加"溶解氧"测量为例：

#### 1. 创建服务模块

```cpp
// include/dissolved_oxygen_service.h
#ifndef DISSOLVED_OXYGEN_SERVICE_H
#define DISSOLVED_OXYGEN_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

// 服务初始化
void AppDOInit(uint32_t *pBuffer, uint32_t bufferSize);
// 数据读取
void AppDOISR(uint32_t *pData, uint32_t *pCount);
// 结果显示
void DOShowResult(uint32_t *pData, uint32_t count);

#ifdef __cplusplus
}
#endif

#endif // DISSOLVED_OXYGEN_SERVICE_H
```

#### 2. 实现测量逻辑

参考 `ph_service.cpp` 实现：
- AD5940 配置（选择 LP Loop 或 HS Loop）
- 序列器生成
- 数据处理算法

#### 3. 集成到主状态机

在 `main.cpp` 中：
- 添加状态枚举 `STATE_DO_INIT`、`STATE_DO_MEASURE`
- 添加命令解析 "do init"、"do read"
- 在 `loop()` 的 switch 中添加状态处理

### 修改测量参数

#### 电导率测量频率修改

默认频率 50 kHz，如需修改为 10 kHz：

1. **修改默认配置**（`Conductivity_service.cpp:28`）：
   ```cpp
   AppCondCfg.SinFreq = 10000.0;  // 10 kHz
   ```

2. **或通过命令动态修改**：
   ```cpp
   // 在 handleSerialCommand 中添加新命令
   if (cmd == "cond freq 10000") {
       AppCondCfg.SinFreq = 10000.0;
       AppCondCfg.CondInited = FALSE;  // 强制重新初始化
   }
   ```

#### 温度测量速率修改

ADS124S08 数据速率在 `ads124s08_drv.cpp:130`：
```cpp
// DATARATE 寄存器配置
regs[DATARATE] = 0x14;  // 20 SPS
// 可选值：0x00(2.5SPS) ~ 0x16(2000SPS)
```

### 自定义通信协议

#### 1. 二进制协议示例

修改 `host_link.cpp` 实现二进制协议：

```cpp
// 定义协议帧结构
#pragma pack(push, 1)
struct MeasurementFrame {
    uint8_t header[2];      // 0xAA 0x55
    uint16_t device_id;
    float temperature;
    float conductivity;
    float ph_current;
    uint32_t timestamp;
    uint16_t crc;
};
#pragma pack(pop)

// 在 loop() 中周期性发送
void sendBinaryFrame() {
    MeasurementFrame frame;
    // 填充数据...
    Serial.write((uint8_t*)&frame, sizeof(frame));
}
```

#### 2. JSON 格式输出

利用 ArduinoJson 库：

```cpp
#include <ArduinoJson.h>

void sendJsonData() {
    StaticJsonDocument<256> doc;
    doc["temp"] = temperature;
    doc["cond"] = conductivity;
    doc["ph"] = ph_current;
    doc["timestamp"] = millis();

    serializeJson(doc, Serial);
    Serial.println();
}
```

### 扩展传感器通道

#### 增加 ISFET 通道数

当前支持 8 通道（3-bit MUX），如需扩展至 16 通道：

1. **硬件**：增加 MUX 级联或使用 4-bit MUX
2. **软件**：修改 `mux_iface.cpp`：
   - 增加 GPIO 地址线
   - 修改 `ChooseISFETChannel()` 支持 1~16 通道

#### 添加新传感器类型

通过 MUX-A 的 CH4（预留）接入新传感器：

1. **硬件连接**：传感器→MUX-A CH4（S4A/S4B）
2. **软件配置**：
   ```cpp
   // 切换通道
   ChooseSenesingChannel(4);  // CH4
   // 复用 AD5940 配置（根据传感器类型选择 HS/LP Loop）
   ```

### 性能优化建议

#### 1. 减少测量延迟

- 电导率测量：降低 DFT 点数（8192→4096），牺牲频率分辨率
- 温度测量：提高 ADS124S08 数据速率（20 SPS→90 SPS）
- pH 测量：减少 SINC3 OSR（4→2），增加噪声但加快响应

#### 2. 降低功耗

- 空闲时关闭 AD5940 高速时钟（32MHz→16MHz）
- 延长测量间隔（ODR 20Hz→1Hz）
- 深度睡眠模式（需硬件支持唤醒源）

#### 3. 提高精度

- 温度测量：使用更高精度参比电阻（0.01%）
- 电导率：增加扫频点数，进行复数拟合
- pH：实现温度补偿 Nernst 方程

### 调试与测试

#### 单元测试

为服务模块编写单元测试：

```cpp
// test_temp_service.cpp
void testResistanceToTemp() {
    TempService ts(ads_drv);
    double r = 1097.32;  // 25°C 附近
    double t = ts.resistanceToTemp(r);
    assert(fabs(t - 25.0) < 0.1);
}
```

#### 硬件在环测试

使用电阻、电容网络模拟传感器：
- 电导率：用精密电阻模拟溶液阻抗
- 温度：用可调电阻模拟 PT1000
- pH：用电流源模拟电极电流

### 贡献指南

1. **代码规范**：
   - 遵循 Arduino 风格（小写+下划线函数名）
   - 头文件使用 C++ 兼容的 `extern "C"` 包装
   - 添加必要的注释（Doxygen 风格）

2. **提交更改**：
   - 创建功能分支
   - 更新相关文档
   - 提交 Pull Request

3. **测试要求**：
   - 新增功能需提供测试用例
   - 确保现有功能不受影响
   - 更新 `_docs/` 中的相关文档

---

## 附录

### A. GPIO 引脚分配表

| GPIO | 功能 | 方向 | 说明 |
|------|------|------|------|
| 4 | DRDY_ADS124S08 | 输入 | ADS124S08 数据就绪，低有效 |
| 12 | SPI_MISO | 输入 | VSPI 总线 MISO |
| 13 | SPI_MOSI | 输出 | VSPI 总线 MOSI |
| 14 | SPI_SCLK | 输出 | VSPI 总线时钟 |
| 15 | RESET_ADS124S08 | 输出 | ADS124S08 硬件复位，低脉冲 |
| 16 | CS_ADS124S08 | 输出 | ADS124S08 片选，低有效 |
| 17 | CHANNEL_MUX_ADDR0 | 输出 | 传感通道 MUX 地址位 0 |
| 18 | CHANNEL_MUX_ADDR1 | 输出 | 传感通道 MUX 地址位 1 |
| 25 | ISFET_MUX_ADDR2 | 输出 | ISFET MUX 地址位 2 |
| 26 | RESET_AD5941 | 输出 | AD5941 硬件复位，低有效 |
| 27 | CS_AD5941 | 输出 | AD5941 片选，低有效 |
| 32 | ISFET_MUX_ADDR0 | 输出 | ISFET MUX 地址位 0 |
| 33 | ISFET_MUX_ADDR1 | 输出 | ISFET MUX 地址位 1 |
| TX0/RX0 | UART0 | 双向 | USB-UART，115200 bps |

### B. 命令速查表

| 类别 | 命令 | 功能 |
|------|------|------|
| **温度** | `temp read` | 读取温度值 |
| | `temp resistance` | 读取原始电阻值 |
| | `temp cal 25` | 记录 25°C 校准点 |
| | `temp cal 35` | 记录 35°C 校准点 |
| | `temp cal 50` | 记录 50°C 校准点 |
| | `temp save` | 保存温度校准系数 |
| | `temp reset` | 清除温度校准 |
| **电导率** | `cond init` | 初始化电导率服务 |
| | `cond read` | 单次电导率测量 |
| | `cond sweep` | 频率扫描（EIS） |
| | `cond cal` | 电极常数校准 |
| **pH** | `ph init` | 初始化 pH 服务 |
| | `ph read` | 单次 pH 电流测量 |
| | `ph cal offset` | 零点校准 |
| | `ph cal gain <R>` | 增益校准（接电阻 R） |
| **系统** | `id <编号>` | 设置设备 ID |
| | `factory reset` | 恢复出厂设置 |

### C. 校准流程摘要

#### 温度三点校准

1. 准备恒温浴（25°C、35°C、50°C）
2. 在每个温度点执行：
   ```
   temp cal 25   （或 35/50）
   ```
3. 三点采集完成后：
   ```
   temp save
   ```

#### 电导率电极常数校准

1. 准备已知电导率的标准液（如 1413 μS/cm KCl 溶液）
2. 浸入电极，执行：
   ```
   cond cal
   ```

#### pH 两点校准

1. 零点校准（pH=7 缓冲液）：
   ```
   ph cal offset
   ```
2. 增益校准（接精密电阻 R）：
   ```
   ph cal gain 1000.0
   ```

### D. 版本历史

| 版本 | 日期 | 更新说明 |
|------|------|---------|
| v1.0 | 2026-02-22 | 初始版本，基于当前固件分析 |
| | | 包含完整用户手册与开发指南 |

### E. 技术支持

- **文档更新**：查看 `_docs/` 目录获取最新文档
- **问题反馈**：提交 GitHub Issue
- **社区讨论**：加入项目 Discussions

---

*本文档根据 AquaMonitor 固件源代码分析整理，适用于用户操作、系统集成与二次开发。文档内容基于代码版本：AquaMonitor 主分支（分析日期 2026-02-22）。*

