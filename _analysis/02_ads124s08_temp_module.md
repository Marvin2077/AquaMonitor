# ADS124S08 温度测量模块分析

> 分析日期：2026-02-22
> 涉及文件：
> - [src/temp_service.cpp](../src/temp_service.cpp) — 温度服务实现（220 行）
> - [include/temp_service.h](../include/temp_service.h) — 温度服务接口（70 行）
> - [src/ads124s08_drv.cpp](../src/ads124s08_drv.cpp) — ADS124S08 SPI 驱动（253 行）
> - [include/ads124s08_drv.h](../include/ads124s08_drv.h) — 驱动接口与类定义（129 行）
> - [src/ads124s08_board_glue.cpp](../src/ads124s08_board_glue.cpp) — ESP32 平台适配（75 行）
> - [include/ads124s08_board_glue.h](../include/ads124s08_board_glue.h) — 适配层接口
> - [include/spi_hal.h](../include/spi_hal.h) — SPI 设备抽象层
> - [src/main.cpp](../src/main.cpp) — 引脚定义与初始化（670 行）

---

## 模块概述

本模块使用 **TI ADS124S08**（24 位精密 Σ-Δ ADC）搭配 **PT1000 铂热电阻（RTD）** 测量水体温度。

### 功能分层

```
┌──────────────────────────────────────────────────────────────┐
│               应用层  TempService（C++ 类）                   │
│  measure() → readResistance() → resistanceToTemp()          │
│                              → applyCalib()                 │
│  recordCalibPoint() + finishCalibration()（三点多项式校准）  │
├──────────────────────────────────────────────────────────────┤
│             ADC 驱动层  ADS124S08_Drv（C++ 类）              │
│  softReset / start / stop                                   │
│  readRegisters / writeRegisters / writeRegisterMasked       │
│  waitDRDY / readDataRaw / readData                          │
│  defaultConfig（寄存器初始化）                               │
├──────────────────────────────────────────────────────────────┤
│             平台适配层  BoardGlue（ads124s08_board_glue）     │
│  make_ads_hal()：将 ESP32 SPI 操作封装为 6 个 HAL 函数指针   │
├──────────────────────────────────────────────────────────────┤
│             SPI 抽象层  SpiDevice / SpiHAL                   │
│  Arduino SPI.beginTransaction / transfer / endTransaction   │
└──────────────────────────────────────────────────────────────┘
```

---

## 传感器类型与连接方式

### 传感器：PT1000 铂热电阻

| 属性 | 参数 |
|------|------|
| 类型 | **铂热电阻 RTD（Resistance Temperature Detector）** |
| 标称值 | **R₀ = 1000 Ω**（0°C 时） |
| 测量范围 | 约 -200°C ～ +850°C（Callendar-Van Dusen 方程有效范围） |
| 故障检测范围 | 100 Ω ～ 5000 Ω（代码硬保护） |
| 温度系数 | A = 3.9083×10⁻³ /°C，B = −5.775×10⁻⁷ /°C²（IEC 60751 标准值）|

### 连接方式（比率法，Ratiometric Measurement）

```
                      ┌─────────────┐
      ADS124S08        │             │
   ┌──────────────┐    │  R_ref      │
   │ IDAC → AIN0 ├────┤  3300 Ω     │
   │              │    │  (参比电阻) │
   │              │    └──────┬──────┘
   │ AIN1 (V+)   ├──────────┤  ← PT1000 高端
   │              │          │  PT1000
   │ AIN2 (V-)   ├──────────┤  ← PT1000 低端
   │              │          │
   │ (AGND)      ├──────────┘
   └──────────────┘

电流路径：IDAC(500μA) → R_ref(3300Ω) → PT1000 → AGND
电压测量：AIN1 - AIN2（PT1000 两端电压差）
```

**关键优势（比率法）：**
- ADC 参考电压 = I × R_ref（与激励电流成正比）
- PT1000 电压 = I × R_PT1000
- 比率 = R_PT1000 / R_ref → 消除 IDAC 绝对精度误差和温漂

> **注意**：代码注释提到 `IDAC → AIN0`，但 `IDACMUX = 0xF0` 的实际编码尚需与手册核验（见已知问题 I1）。

---

## ADC 配置参数

### SPI 硬件配置（`main.cpp:20-30` + `spi_hal.h`）

| 参数 | 值 | 说明 |
|------|---|------|
| SPI 总线 | ESP32 VSPI（共用） | SCLK=GPIO14, MISO=GPIO12, MOSI=GPIO13 |
| ADS124S08 CS | GPIO **16** | 低有效 |
| ADS124S08 RESET | GPIO **15** | 低脉冲复位，≥10μs 低，2ms 稳定 |
| ADS124S08 DRDY | GPIO **4** | 低有效，数据就绪信号 |
| SPI 时钟频率 | **4 MHz** | ADS124S08 最高支持 32MHz，此处保守设置 |
| SPI 模式 | **SPI_MODE1** | CPOL=0, CPHA=1（下降沿采样）|
| 位序 | **MSBFIRST** | |

> AD5940 使用同一总线但 SPI_MODE0、8MHz，通过不同 CS 引脚隔离。

### ADS124S08 寄存器配置（`defaultConfig`，`ads124s08_drv.cpp:114`）

| 寄存器 | 地址 | 写入值 | 位域解析 | 说明 |
|--------|------|--------|---------|------|
| `INPMUX` | 0x02 | `0x12` | MUXP[7:4]=0001（AIN1），MUXN[3:0]=0010（AIN2）| **差分输入：AIN1-AIN2** |
| `PGA` | 0x03 | `0x09` | PGA_EN=1，GAIN[2:0]=001 | PGA 使能，**增益=2×** |
| `DATARATE` | 0x04 | `0x14` | MODE=0（连续），FILTER=0（低延迟），DR=0100 | **连续转换，20 SPS** |
| `REF` | 0x05 | `0x02` | （TODO：待按手册核验） | 参考选择配置 |
| `IDACMAG` | 0x06 | `0x05` | IMAG[3:0]=0101 | **IDAC 激励电流 = 500 μA** |
| `IDACMUX` | 0x07 | `0xF0` | I1MUX[7:4]=0xF，I2MUX[3:0]=0x0 | IDAC 输出路由（⚠️ 见已知问题）|
| `SYS` | 0x09 | `0x10` | SENDSTAT=0，CRC=0 | 关闭状态字节和 CRC 校验 |

**完整参数汇总（含 main.cpp 中的 TempService::Config）：**

| 参数 | 值 | 影响 |
|------|---|------|
| `Rref_ohm` | **3300 Ω** | 参比电阻值，决定量程与灵敏度 |
| `pga_gain` | **2** | 与 PGA 寄存器配置一致 |
| IDAC 电流 | **500 μA** | R_ref 上产生 500μA×3300Ω=1.65V 参考 |
| 数据速率 | **20 SPS** | 每次转换周期 50ms，DRDY 超时设为 100ms |

---

## 温度计算方法

### 完整计算链

```
ADS124S08 输出
    │
    │ 24-bit 有符号整数（符号扩展 bit[23]→bit[31]）
    ▼
codeToResistance(code24)
    │  ratio = code24 / 8388608.0  (2^23 = FS)
    │  R = ratio × Rref(3300Ω) / gain(2)
    ▼
resistanceToTemp(R)               ← Callendar-Van Dusen 方程
    │  T ≥ 0°C：解一元二次方程（解析解）
    │  T < 0°C：牛顿迭代法（10次迭代）
    ▼
applyCalib(t_raw)
    │  若有校准系数：T_out = a×t² + b×t + c
    │  若无校准系数：T_out = t_raw（直通）
    ▼
out_temp（double，°C）
```

### 第一步：码值转电阻（比率法）

```cpp
// temp_service.cpp:59-70
double ratio = code24 / 8388608.0;    // FS = 2^23
double R = ratio × 3300.0 / 2.0;      // Rref=3300Ω，PGA_gain=2
```

**原理：**
```
ADC 测量量 = V_PT1000 / V_ref = (I × R_PT1000) / (I × R_ref)
           = R_PT1000 / R_ref

code / FS = V_diff / V_ref × gain = (R_PT1000 / R_ref) × gain

→ R_PT1000 = (code / FS) × R_ref / gain
```

### 第二步：电阻转温度（Callendar-Van Dusen）

**IEC 60751 PT1000 标准系数：**

| 常数 | 值 | 适用范围 |
|------|---|---------|
| R₀ | 1000 Ω | 0°C 基准 |
| A | 3.9083 × 10⁻³ /°C | 全范围 |
| B | −5.775 × 10⁻⁷ /°C² | 全范围 |
| C | −4.183 × 10⁻¹² /°C⁴ | 仅 T < 0°C |

**T ≥ 0°C（解析解，`temp_service.cpp:80-90`）：**
```
R(T) = R₀ × (1 + A×T + B×T²)

整理成标准二次方程：
B×T² + A×T + (1 - R/R₀) = 0

令 Z = 1 - R/R₀，用求根公式：
T = (-A + √(A² - 4B×Z)) / (2B)
  = (-A + √(A² - 4B×(1-R/R₀))) / (2B)
```

**T < 0°C（Newton-Raphson 迭代，`temp_service.cpp:93-103`）：**
```
R(T) = R₀ × (1 + A×T + B×T² + C×(T-100)×T³)

牛顿法：
  初始猜测 T₀ = -10°C
  每次迭代：
    R_guess(T) = R₀×(1 + A×T + B×T² + C×(T-100)×T³)
    R'(T)      = R₀×(A + 2B×T + C×(4T³ - 300T²))
    T_next     = T - (R_guess - R_measured) / R'(T)
  收敛条件：|R_guess - R_measured| < 0.001 Ω
  最大迭代：10 次
```

### 第三步：三点多项式校准（`temp_service.cpp:162-219`）

**校准模型：**
```
T_true = a × T_meas² + b × T_meas + c
（二次多项式拟合，补偿系统误差）
```

**校准流程：**
```
1. 在稳定温度下用标准温度计读三个点
   默认点（main.cpp:205-230）：25°C、35°C、50°C

2. 每个校准点：10次平均（readResistance×10，间隔50ms）
   temp_points_[index].t_meas = avg(resistanceToTemp(R))
   temp_points_[index].t_true = 用户输入真实值

3. 构造 3×3 线性方程组：
   ┌ T_meas₁²  T_meas₁  1 ┐ ┌ a ┐   ┌ T_true₁ ┐
   │ T_meas₂²  T_meas₂  1 │×│ b │ = │ T_true₂ │
   └ T_meas₃²  T_meas₃  1 ┘ └ c ┘   └ T_true₃ ┘

4. 高斯消元法求解（solve3x3，temp_service.cpp:10-25）
   防奇异性检查：三点 t_meas 差异 ≥ 0.1°C，主元 ≥ 1e-12

5. 系数存入 CalibCoeff{a, b, c, valid=true}
   持久化保存到 Flash（saveTempParams()）
```

---

## 与主控的通信协议

### SPI 通信协议（ADS124S08 指令集）

**帧格式汇总：**

| 操作 | 发送字节 | 接收字节 | 备注 |
|------|---------|---------|------|
| 软复位 | `[0x06]` | — | CS 包围 |
| 开始转换 | `[0x08]` | — | 开始连续转换 |
| 停止转换 | `[0x0A]` | — | |
| 读 n 个寄存器 | `[0x20\|addr, n-1]` | `[n字节数据]` | CS 贯穿整个操作 |
| 写 n 个寄存器 | `[0x40\|addr, n-1, data...]` | — | |
| 读数据 | `[0x12, 0xFF, 0xFF, 0xFF]` | `[_, B2, B1, B0]` | 24-bit MSB first |

**RDATA 时序（`ads124s08_drv.cpp:93-103`）：**
```
CS_LOW
→ SPI_TX: [0x12]                    // RDATA 命令
→ delay 10μs
→ SPI_RX: [b[0], b[1], b[2]]        // 3字节 = 24位
CS_HIGH

raw24 = (b[0]<<16) | (b[1]<<8) | b[2]    // MSB first
signed = (raw24 & 0x800000) ? raw24|0xFF000000 : raw24  // 符号扩展
```

**RREG 时序（`ads124s08_drv.cpp:20-32`）：**
```
CS_LOW
→ SPI_TX: [(0x20|addr), (n-1)]      // 命令头：2字节
→ SPI_RX: [data × n]                // 寄存器数据
CS_HIGH
```

> **SPI 事务注意**：`spi_txrx` 回调每次调用都会执行 `beginTransaction/endTransaction`，在单次 CS 断言内可能多次调用，ESP32 上不会产生总线冲突，但会产生额外的函数调用开销。

### DRDY 等待机制（`ads124s08_drv.cpp:74-88`）

```cpp
// 轮询 GPIO，低电平有效，超时 100ms（TempService 调用时）
while (digitalRead(pin_drdy) != 0) {
    if (millis() - start > timeout_ms) return false;
    yield();  // 让出 ESP32 CPU 时间片
}
```

- 20 SPS 时数据周期 = 50ms，超时设为 100ms（2 倍余量）
- 无硬件中断，纯轮询

### 与上层控制的接口（串口命令，`main.cpp:469-508`）

| 串口指令 | 触发动作 | 状态机转换 |
|---------|---------|-----------|
| `temp read` | 读一次温度 | `STATE_TEMP_MEASURE` |
| `temp cal 25` | 记录 25°C 校准点 | `STATE_TEMP_CAL_P1` |
| `temp cal 35` | 记录 35°C 校准点 | `STATE_TEMP_CAL_P2` |
| `temp cal 50` | 记录 50°C 校准点 | `STATE_TEMP_CAL_P3` |
| `temp save` | 计算并生效校准 | `STATE_TEMP_SAVE_CAL` |
| `temp reset` | 清除校准系数 | `STATE_TEMP_RESET_CAL` |
| `temp resistance` | 读原始电阻值（调试）| `STATE_TEMP_RESISTANCE` |

### 校准数据持久化（`main.cpp:169-180` + `storage_manager`）

```cpp
// 上电时从 Flash 读取：
TempCalibData tempData = loadTempParams();
g_tempSvc->setCalib({a, b, c, valid});

// 完成校准时写入 Flash：
saveTempParams(c.a, c.b, c.c, true);
```

---

## 模块设计亮点

1. **依赖注入（DI）模式**：`TempService` 通过构造函数接收 `ADS124S08_Drv&` 引用，便于单元测试和多实例复用
2. **HAL 回调模式**：`ADS124S08_Drv` 通过 6 个函数指针组成的 `Hal` 结构体解耦硬件，平台适配只需实现 `make_ads_hal()`
3. **双区间公式**：T≥0 用解析解（快速精确），T<0 用 Newton-Raphson（适应非线性），边界在 R₀=1000Ω 处平滑过渡
4. **3 点多项式校准**：用二次多项式拟合仪器误差，比单点偏移校准更精确，比查表法更省 RAM

---

## 已知问题与注意事项

| 编号 | 位置 | 问题描述 | 严重程度 |
|------|------|---------|---------|
| I1 | `defaultConfig:160` | `IDACMUX=0xF0`：若编码 0x0=AIN0，0xF=NC，则实际是 I1MUX→NC、I2MUX→AIN0，与注释"I1MUX→AIN0"**相反** | **高** |
| I2 | `defaultConfig:146` | `REF=0x02`：注释为 TODO，参考选择和缓冲使能位域需按手册核验 | 中 |
| I3 | `defaultConfig:130` | `PGA=0x09`：注释说 PGA_EN=1, GAIN=2x，但具体位图需核验，若 GAIN[2:0]=001 不代表 2× 则增益有误 | 中 |
| I4 | `readPT1000Temperature` (ads124s08_drv.cpp:214) | 独立函数体不完整（无 `temperature` 赋值和 `return`，UB），已被 `TempService` 中更完整的实现取代 | 低（废弃）|
| I5 | `main.cpp:205-230` | 三个校准温度值硬编码为 25/35/50°C，若实际校准温度不同会写入错误真值 | 中 |
| I6 | `solve3x3` | 仅检查对角主元，若矩阵病态（三点接近）可能产生大误差而无提示 | 低 |
