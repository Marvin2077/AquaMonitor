# 04 ESP32 主控制器分析

> 分析范围：`src/main.cpp`, `src/spi_hal.cpp`, `src/mux_iface.cpp`, `src/bt_service.cpp`, `src/storage_manager.cpp`, `src/host_link.cpp`（空文件），及对应 `include/` 头文件
> 构建系统：PlatformIO (`platformio.ini`)

---

## ESP32 型号确认

| 项目 | 值 |
|------|-----|
| **PlatformIO Board ID** | `esp32dev` |
| **平台** | `espressif32` |
| **框架** | Arduino |
| **Flash 分区** | `huge_app.csv`（大应用分区） |
| **监视器波特率** | 115200 |
| **上传波特率** | 921600 |

> **结论**：标准双核 ESP32（ESP32-D0WDQ6 / esp32dev），**非 ESP32-PICO**。使用 Arduino 框架，依赖 `ArduinoJson ^6.21.0`。

---

## ESP32 外设配置

### 1. SPI 总线（共享总线，VSPI）

```
SpiHAL::beginBus(PIN_SCLK=14, PIN_MISO=12, PIN_MOSI=13)
```

| 引脚 | GPIO | 功能 |
|------|------|------|
| SCLK | 14 | SPI 时钟 |
| MISO | 12 | 主机输入 |
| MOSI | 13 | 主机输出 |

两颗 AFE 芯片挂载同一 SPI 总线，通过各自片选区分：

#### AD5941（阻抗/电导率/pH 前端）
| 参数 | 值 |
|------|-----|
| CS 引脚 | GPIO 27 |
| RESET 引脚 | GPIO 26 |
| SPI 速率 | 8 MHz |
| 位序 | MSBFIRST |
| SPI 模式 | MODE0 |

#### ADS124S08（温度 ADC）
| 参数 | 值 |
|------|-----|
| CS 引脚 | GPIO 16 |
| RESET 引脚 | GPIO 15 |
| DRDY 引脚 | GPIO 4 |
| SPI 速率 | 4 MHz |
| 位序 | MSBFIRST |
| SPI 模式 | MODE1 |

**SPI HAL 封装**（`spi_hal.h`，纯 inline 实现）：

```cpp
struct SpiDevice { cs_pin, clk_hz, bit_order, spi_mode, SPISettings };
// 方法：begin() / cs_low() / cs_high() / beginTxn() / endTxn() / transfer()
namespace SpiHAL { beginBus(), delay_us(), delay_ms() }
```

---

### 2. UART（USB 串口）

| 参数 | 值 |
|------|-----|
| 接口 | `Serial`（UART0，USB-UART 桥） |
| 波特率 | 115200 |
| 用途 | 命令下行 + 数据/日志上行 |
| 帧格式 | ASCII 文本，`\n` 结尾 |

---

### 3. 蓝牙（已实现，main.cpp 暂未集成）

| 参数 | 值 |
|------|-----|
| 库 | `BluetoothSerial`（ESP32 Arduino Core 内置） |
| 类型 | Classic Bluetooth SPP |
| API | `BTService_Init(deviceName)` / `BTService_ReadCommand(cmdBuffer)` |
| 帧格式 | ASCII 文本，`\n` 结尾（与串口协议一致） |

> `bt_service.cpp` 已完整实现（init + 按行读取），但 `main.cpp` 中当前未调用，属于**备用/扩展通道**。

---

### 4. GPIO（数字输出 — MUX 控制）

#### 传感通道 MUX（2 位地址，4 路选 1）
| 引脚 | GPIO | 说明 |
|------|------|------|
| CHANNEL_MUX_ADDR0 | 17 | MUX 地址位 0 |
| CHANNEL_MUX_ADDR1 | 18 | MUX 地址位 1 |

| ADDR1 | ADDR0 | 通道 | 传感器 |
|-------|-------|------|--------|
| 0 | 0 | S1A/S1B | 阻抗（电导率） |
| 0 | 1 | S2A/S2B | 余氯 |
| 1 | 0 | S3A/S3B | pH |

#### ISFET 通道 MUX（3 位地址，8 路选 1）
| 引脚 | GPIO | 说明 |
|------|------|------|
| ISFET_MUX_ADDR0 | 32 | MUX 地址位 0 |
| ISFET_MUX_ADDR1 | 33 | MUX 地址位 1 |
| ISFET_MUX_ADDR2 | 25 | MUX 地址位 2 |

支持 1–8 路 ISFET 传感器通道切换。

---

### 5. NVS Flash 存储（`Preferences` 库）

| 参数 | 值 |
|------|-----|
| 库 | `Preferences`（ESP32 NVS 封装） |
| 命名空间 | `"dev"` |

| Key | 类型 | 含义 |
|-----|------|------|
| `"id"` | int | 设备 ID |
| `"cond_k"` | float | 电导率电极常数 K |
| `"ph_off"` | uint16 | pH 零点偏移 Code |
| `"ph_rtia"` | float | pH RTIA 增益电阻值（Ω） |
| `"temp_a"` | float | 温度校准系数 A |
| `"temp_b"` | float | 温度校准系数 B |
| `"temp_c"` | float | 温度校准系数 C |
| `"temp_v"` | bool | 温度校准是否有效 |

---

## 系统架构（任务/状态机）

### 整体架构

- **单线程 Arduino 模型**（无 FreeRTOS 任务，无抢占）
- `setup()` 顺序初始化所有外设
- `loop()` 每次迭代：① 处理串口命令 → ② 定时器检查（500ms） → ③ 状态机执行

```
loop():
  ├─ handleSerialCommand()   // 解析命令，转换状态
  ├─ if(millis - last > 500ms)  // 定时触发（当前体为空）
  └─ switch(currentState)    // 执行当前状态动作
```

---

### 有限状态机（FSM）

```
                   ┌──────────────────┐
                   │   STATE_IDLE     │◄─── 所有状态执行完毕后返回
                   └────────┬─────────┘
           串口命令触发      │
     ┌─────────────────────┴──────────────────────┐
     │                                            │
  温度指令                                     电导率/pH 指令
     │                                            │
  ┌──┴────────────────────┐          ┌────────────┴──────────────┐
  │ STATE_TEMP_MEASURE    │          │ STATE_COND_INIT           │
  │ STATE_TEMP_CAL_P1     │          │ STATE_COND_MEASURE        │
  │ STATE_TEMP_CAL_P2     │          │ STATE_COND_SWEEP          │
  │ STATE_TEMP_CAL_P3     │          │ STATE_COND_CAL            │
  │ STATE_TEMP_SAVE_CAL   │          │ STATE_PH_INIT             │
  │ STATE_TEMP_RESET_CAL  │          │ STATE_PH_MEASURE          │
  │ STATE_TEMP_RESISTANCE │          │ STATE_PH_CAL_OFFSET       │
  └───────────────────────┘          │ STATE_PH_CAL_GAIN         │
                                     └───────────────────────────┘
```

#### 状态列表（共 15 个）

| 状态枚举 | 触发命令 | 动作 |
|---------|---------|------|
| `STATE_IDLE` | — | 等待 |
| `STATE_TEMP_MEASURE` | `temp read` | 读取 RTD 温度并打印 |
| `STATE_TEMP_CAL_P1` | `temp cal 25` | 记录 25°C 校准点 |
| `STATE_TEMP_CAL_P2` | `temp cal 35` | 记录 35°C 校准点 |
| `STATE_TEMP_CAL_P3` | `temp cal 50` | 记录 50°C 校准点 |
| `STATE_TEMP_SAVE_CAL` | `temp save` | 三点拟合计算 a/b/c，写 NVS |
| `STATE_TEMP_RESET_CAL` | `temp reset` | 清除温度校准 |
| `STATE_TEMP_RESISTANCE` | `temp resistance` | 读取原始 RTD 电阻（Ω） |
| `STATE_COND_INIT` | `cond init` | 初始化 AD5941 电导率序列器 |
| `STATE_COND_MEASURE` | `cond read` | 单次 1kHz 阻抗测量 |
| `STATE_COND_SWEEP` | `cond sweep` | 频率扫描 1kHz→100kHz（多点） |
| `STATE_COND_CAL` | `cond cal` | 用标准液校准电极常数 K，写 NVS |
| `STATE_PH_INIT` | `ph init` | 初始化 AD5941 pH 序列器 |
| `STATE_PH_MEASURE` | `ph read` | 单次 pH 测量 |
| `STATE_PH_CAL_OFFSET` | `ph cal offset` | 校准 ADC 零点 Code，写 NVS |
| `STATE_PH_CAL_GAIN` | `ph cal gain <R>` | 用外接已知电阻校准 RTIA，写 NVS |

#### 模式互斥

- `g_isCondMode` 与 `g_ispHMode` 是互斥标志：
  - 进入 `STATE_COND_INIT` 时：`g_isCondMode=true`, `g_ispHMode=false`, `AppPHCfg.PHInited=FALSE`
  - 进入 `STATE_PH_INIT` 时：`g_ispHMode=true`, `g_isCondMode=false`, `AppCondCfg.CondInited=FALSE`
- 执行测量/校准前会检查对应标志，未初始化则报错并返回 IDLE

---

## 通信协议规范

### 信道

| 信道 | 协议 | 当前状态 |
|------|------|---------|
| USB UART（`Serial`） | 115200-8N1，ASCII 文本 | **主信道，已集成** |
| 蓝牙 SPP（`SerialBT`） | Classic BT，ASCII 文本 | **已实现，未集成到 main** |

### 帧格式

```
命令帧（上位机 → ESP32）：
  <command_string>\n

响应帧（ESP32 → 上位机）：
  <text_line>\n     // 一条或多条纯文本行
```

- **纯文本 ASCII**，无固定包头/包尾/CRC
- 命令以 `\n`（LF）结尾，`trim()` 后匹配
- 响应为 `Serial.println()` / `Serial.printf()`，换行分隔
- 无握手、无应答确认机制（单向无返回确认）

---

## 命令与响应格式

### 温度类

| 命令 | 响应示例 |
|------|---------|
| `temp read` | `[CMD] Measuring Temperature` → ` Water Temp: 25.123456 ` |
| `temp resistance` | `[CMD] Reading raw resistance...` → `Resistance: 109.45 Ohm` |
| `temp cal 25` | `[CMD] Measuring Point 1 (25.0 C)... Keep sensor stable.` → `Point 1 Saved!` |
| `temp cal 35` | → `Point 2 Saved!` |
| `temp cal 50` | → `Point 3 Saved!` |
| `temp save` | `[CMD] Computing temperature calibration coefficients...` → `Calibration DONE! a=..., b=..., c=...` |
| `temp reset` | `[CMD] Resetting temperature calibration coefficients...` → `Calibration Cleared.` |

### 电导率类

| 命令 | 响应示例 |
|------|---------|
| `cond init` | `[CMD] Init conductivity service...` → `Conductivity Service Init OK!` |
| `cond read` | `[CMD] Triggering Cond_Impedance Measurement...` → `Conductivity: 1413.0000` |
| `cond cal` | `[CMD] Starting K_Cell calibration` → `>>> Calibration Successful! <<<` + K_Cell 值 |
| `cond sweep` | `[CMD] Starting Frequency Sweep (1kHz -> 100kHz)...` → 多行结果 → `>>> Sweep Completed! <<<` |

### pH 类

| 命令 | 响应示例 |
|------|---------|
| `ph init` | `[CMD] Initializing pH Measurement...` → `pH Service Init OK!` |
| `ph read` | `[CMD] Triggering Single pH Measurement...` → （由 `PHShowResult()` 输出） |
| `ph cal offset` | `[CMD] Calibrating pH Offset...` → `>>> Offset Calibrated! New Zero Code: 0x8000 (32768) <<<` |
| `ph cal gain <ohms>` | `[CMD] Calibrating Gain with R_ext = 1000.0 Ohm...` → `>>> Gain Calibrated! ... New RTIA: 985.23 Ohm <<<` |

### 系统类

| 命令 | 响应示例 |
|------|---------|
| `id <number>` | `Device ID saved: 1`（仅在启动时 ID 未设置时触发阻塞循环） |
| `factory reset` | `[CMD] Resetting to Factory Defaults...` → 各参数重置信息 → `>>> Factory Reset Complete. <<<` |

### 错误响应

| 错误情况 | 响应 |
|---------|------|
| 在非电导率模式下发 `cond read/cal/sweep` | `Not in Conductivity Mode` |
| 在非 pH 模式下发 `ph read/cal*` | `Not in pH Mode` |
| 电导率/pH 服务未初始化 | `Conductivity/pH Service haven't been initialized` |
| ADS124S08 初始化失败 | `!!! ADS124S08 configuration FAILED! Halting. !!!`（死循环） |
| pH 增益校准信号太低 | `>>> Error: Signal too low (0.0000V). Is resistor connected? <<<` |

---

## 附录：完整 GPIO 引脚映射表

| GPIO | 功能 | 方向 | 说明 |
|------|------|------|------|
| 4 | DRDY_ADS124S08 | 输入 | ADS124S08 数据就绪中断 |
| 12 | MISO | 输入 | SPI 总线 |
| 13 | MOSI | 输出 | SPI 总线 |
| 14 | SCLK | 输出 | SPI 总线 |
| 15 | RESET_ADS124S08 | 输出 | ADS124S08 复位 |
| 16 | CS_ADS124S08 | 输出 | ADS124S08 片选 |
| 17 | CHANNEL_MUX_ADDR0 | 输出 | 传感通道 MUX 地址位 0 |
| 18 | CHANNEL_MUX_ADDR1 | 输出 | 传感通道 MUX 地址位 1 |
| 25 | ISFET_MUX_ADDR2 | 输出 | ISFET MUX 地址位 2 |
| 26 | RESET_AD5941 | 输出 | AD5941 复位 |
| 27 | CS_AD5941 | 输出 | AD5941 片选 |
| 32 | ISFET_MUX_ADDR0 | 输出 | ISFET MUX 地址位 0 |
| 33 | ISFET_MUX_ADDR1 | 输出 | ISFET MUX 地址位 1 |
| TX0/RX0 | UART0 | 双向 | USB-UART，115200 |

---

*分析时间：2026-02-22*
