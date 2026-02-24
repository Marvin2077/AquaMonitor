# pH 测量模块分析

> 分析日期：2026-02-22
> 涉及文件：
> - [src/ph_service.cpp](../src/ph_service.cpp) — pH 应用服务（393 行）
> - [include/ph_service.h](../include/ph_service.h) — pH 服务接口
> - [src/ads124s08_drv.cpp](../src/ads124s08_drv.cpp) — ADS124S08 ADC 驱动（温度用，253 行）
> - [include/ads124s08_drv.h](../include/ads124s08_drv.h) — ADS124S08 驱动接口
> - [src/ads124s08_board_glue.cpp](../src/ads124s08_board_glue.cpp) — ADS124S08 ESP32 适配层（75 行）
> - [include/ads124s08_board_glue.h](../include/ads124s08_board_glue.h) — 适配层接口

---

## 模块概述

pH 测量模块**不使用 ADS124S08**，而是复用 **AD5940 芯片的低功耗环路（LP Loop）** 进行电流法 pH 测量。

> ADS124S08 专门负责**温度测量**（PT1000 RTD），详见本文 "ADS124S08 温度测量" 章节。

### 测量原理对比

| 参数 | pH 模块（ph_service） | 电导率模块（Conductivity_service） |
|------|----------------------|----------------------------------|
| 芯片 | AD5940 LP Loop | AD5940 HS Loop |
| 信号类型 | **直流（DC）** 电流 | 交流（AC）正弦波阻抗 |
| ADC 路径 | SINC3 滤波输出 | DFT 硬件加速器 |
| FIFO 源 | `FIFOSRC_SINC3` | `FIFOSRC_DFT` |
| FIFO 阈值 | 1（每次1个 uint32_t） | 4（2个复数 DFT）|
| 激励 | LPDAC 直流偏置 | HSDAC 正弦波 |
| 目标参数 | 电极电流 → （待补充 pH 值） | 复数阻抗 → 电导率 |

### pH 电化学原理（Nernst 方程，代码中尚未实现）

```
E_pH = E_ref + S × (pH_ref - pH)
其中：
  S ≈ -59.16 mV/pH  (25°C, Nernst 斜率)
  E_ref = 参比电极电位
  pH = E_ref - E_measured / S + pH_ref
```

> **注意：当前代码只计算了电极电流（μA），尚未实现 pH 值换算。**

---

## 功能分层

```
┌─────────────────────────────────────────────────────┐
│             应用层  ph_service.cpp                   │
│   AppPHInit / AppPHISR / AppPHCtrl / PHShowResult   │
├─────────────────────────────────────────────────────┤
│          平台配置（共用）ad5941PlatformCfg.cpp         │
│          ADI 驱动库（共用）ad5940.c                   │
├─────────────────────────────────────────────────────┤
│          板级胶合层（共用）ad5941_board_glue.cpp       │
│          ESP32 Arduino SPI                          │
└─────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────┐
│       温度测量（独立）ads124s08_drv.cpp               │
│   readPT1000Temperature / defaultConfig             │
├─────────────────────────────────────────────────────┤
│       适配层 ads124s08_board_glue.cpp                │
│   BoardGlue::make_ads_hal()（SPI 回调封装）          │
└─────────────────────────────────────────────────────┘
```

---

## 核心寄存器配置

### 一、LPDAC 配置（低功耗 DAC，生成偏置电压）

```c
// ph_service.cpp:15-33
AppPHCfg.LpdacSel      = LPDAC0;
AppPHCfg.DacData12Bit  = 0x745;     // 12位 DAC：Vbias ≈ 1.135 V
AppPHCfg.DacData6Bit   = 0x17;      // 6位  DAC：Vzero ≈ 0.898 V
AppPHCfg.LpDacRef      = LPDACREF_2P5;  // 参考：内部 2.5V
AppPHCfg.LpDacSrc      = LPDACSRC_MMR; // 源：直接写寄存器（静态）
AppPHCfg.LpDacVbiasMux = LPDACVBIAS_12BIT;  // Vbias 来自 12bit DAC
AppPHCfg.LpDacVzeroMux = LPDACVZERO_6BIT;   // Vzero 来自 6bit DAC
```

**电压计算：**
```
Vbias = 0x745 / 4096 × 2500 mV = 1861/4096 × 2500 ≈ 1135 mV
Vzero = 0x17  / 64   × 2500 mV = 23/64   × 2500 ≈ 898  mV

差值（工作点偏置）= Vbias - Vzero ≈ 1135 - 898 = 237 mV
```

**LPDAC 开关路由（`LpDacSW`）：**
| 开关 | 含义 |
|------|------|
| `VBIAS2LPPA` | Vbias → LP-PA 正极（给电位放大器提供偏置） |
| `VBIAS2PIN`  | Vbias → 外部引脚（连接参比电极） |
| `VZERO2LPTIA`| Vzero → LPTIA 参考节点 |
| `VZERO2PIN`  | Vzero → 外部引脚（连接 CE/对电极） |

---

### 二、LP-PA + LPTIA 配置（低功耗电位放大器 + 跨阻放大器）

```c
// ph_service.cpp:37-46
AppPHCfg.LpAmpSel     = LPAMP0;
AppPHCfg.LpAmpPwrMod  = LPAMPPWR_NORM;    // 正常功耗模式
AppPHCfg.LpPaPwrEn    = bTRUE;            // LP-PA 上电
AppPHCfg.LpTiaPwrEn   = bTRUE;            // LPTIA 上电
AppPHCfg.LpTiaSW      = LPTIASW(12)|LPTIASW(13)|LPTIASW(2)|LPTIASW(10)|LPTIASW(5)|LPTIASW(9);
AppPHCfg.LpTiaRf      = LPTIARF_SHORT;    // RC 滤波电阻：短接（无滤波）
AppPHCfg.LpTiaRtia    = LPTIARTIA_200R;   // RTIA = 200 Ω（量程决定因素）
AppPHCfg.LpTiaRload   = LPTIARLOAD_100R;  // Rload = 100 Ω
```

> LPTIA 反馈电阻 200Ω 决定了电流检测灵敏度：
> `ΔV_out = I × 200Ω`，当 ADC 满量程 ≈ 1.82V 时，最大可测电流约 ±9 mA

---

### 三、HSTIA 配置（高速 TIA，此处做信号路由用）

```c
// ph_service.cpp:48-58
AppPHCfg.HstiaRtiaSel  = HSTIARTIA_160K;   // 160kΩ RTIA（未实际用于 pH）
AppPHCfg.HstiaBias     = HSTIABIAS_1P1;    // 1.1V 偏置
AppPHCfg.HstiaCtia     = 16;               // 16pF
AppPHCfg.DswitchSel    = SWD_OPEN;
AppPHCfg.PswitchSel    = SWP_PL | SWP_PL2;
AppPHCfg.NswitchSel    = SWN_OPEN;
AppPHCfg.TswitchSel    = SWT_AIN1 | SWT_TRTIA;
```

> D/N 开关均为 OPEN，HSTIA 在 pH 模式下主要作为 ADC 测量节点（`ADCMUXP_HSTIA_P`），电流测量路径实际走 LPTIA。

---

### 四、ADC 滤波链配置

**测量序列中（`AppPHSeqMeasureGen`，`ph_service.cpp:219`）：**

```c
// 初始化序列中的滤波配置（AppPHSeqCfgGen:185-193）
adc_filter.ADCRate      = ADCRATE_1P6MHZ;     // ADC 时钟：32MHz / 20 = 1.6MHz
adc_filter.ADCSinc3Osr  = ADCSINC3OSR_4;      // SINC3 过采样率 = 4
adc_filter.ADCSinc2Osr  = ADCSINC2OSR_22;     // SINC2 过采样率 = 22
adc_filter.BpNotch      = bTRUE;              // 旁路 Notch
adc_filter.Sinc2NotchEnable = bTRUE;

// ADC MUX
adc_base.ADCMuxP = ADCMUXP_HSTIA_P;   // 正端：HSTIA 输出（即 LPTIA 输出）
adc_base.ADCMuxN = ADCMUXN_VSET1P1;   // 负端：1.1V 参考
adc_base.ADCPga  = ADCPGA_1;          // PGA 增益 = 1
```

**ADC 信号链：**
```
LP-PA/LPTIA输出 → HSTIA_P 节点 → ADC_P
1.1V Vset      →              → ADC_N
                                 ↓
                         差分输入 = HSTIA_P - 1.1V
                                 ↓
                      SINC3(OSR=4) → SINC2(OSR=22)
                                 ↓
                         FIFOSRC_SINC3 → FIFO
```

---

### 五、FIFO 配置（pH 与电导率的差异）

```c
// ph_service.cpp:282-287
fifo_cfg.FIFOSrc    = FIFOSRC_SINC3;  // ★ 关键：不是 DFT，是 SINC3 输出
fifo_cfg.FIFOThresh = 1;              // 每1个数据触发一次 ISR 查询
```

> 这是 pH 与电导率最关键的区别：pH 测的是**直流电流**，不需要 DFT，直接取 SINC3 滤波后的 ADC 值即可。

---

## 测量流程

### 总体流程图

```
AppPHCfg_init()
  └─ 设置 LPDAC 偏置参数（Vbias=1.135V, Vzero=0.898V）
  └─ 设置 LPTIA Rtia=200Ω，关闭 RC 滤波
  └─ ZeroOffset_Code=32768（默认中点，需校准）
    │
    ▼
AppPHInit(buffer, bufSize)
  ├─ 唤醒 AFE（轮询 ADIID）
  ├─ 配置序列器（2KB SRAM）
  ├─ 重配 FIFO（FIFOSRC_SINC3，阈值=1）
  ├─ AppPHSeqCfgGen() → 写入 SEQID_1（初始化序列）
  │    配置：参考系统 + LPDAC偏置 + LP-PA/LPTIA + HSTIA + ADC滤波链
  ├─ AppPHSeqMeasureGen() → 写入 SEQID_0（测量序列）
  ├─ 触发 SEQID_1，等待 ENDSEQ
  └─ 配置 SEQID_0，等待触发
    │
    ▼（主循环轮询，无 WUPT 定时触发）
AppPHCtrl(PHCTRL_START, 0)
  └─ AD5940_SEQMmrTrig(SEQID_0)  ← 手动触发测量
    │
    ▼ SEQID_0 执行
  1. ADCMux → HSTIA_P / VSET1P1
  2. 上电：ADC + HSTIA + SINC2NOTCH + INAMPPWR + EXTBUFPWR
  3. 等待 16×80 clk（稳定延时 ≈ 80μs @ 16MHz）
  4. 启动 ADC 转换（ADCCNV）
  5. 等待 WaitClks（SINC3 滤波完成）
  6. 停止 ADC 转换 + ADC 下电
    │
    ▼ FIFO 有数据（阈值=1）
AppPHISR(buffer, count)
  ├─ 唤醒 AFE
  ├─ 检查 DATAFIFOTHRESH 标志
  ├─ 读取 FIFO（FIFOCnt 个 uint32_t）
  └─ 返回原始数据（未处理）
    │
    ▼
PHShowResult(data, count)
  └─ 原始码值 → 差分电压 → 电流(μA) → Serial 打印
```

---

### SEQID_1（初始化序列）详解

```
AppPHSeqCfgGen() 生成的序列内容：
  1. 配置参考系统（HP Bandgap ON, HP1V1/1V8 Buf ON, LP Bandgap ON, LP RefBuf ON）
  2. 配置 LPDAC（Vbias=0x745, Vzero=0x17, 开关路由, 参考=2.5V）
  3. 配置 LP-PA（电位放大器上电，闭合反馈开关）
  4. 配置 LPTIA（Rtia=200Ω, Rload=100Ω, 开关配置）
  5. 配置 HSTIA（偏置=1.1V, Rtia=160K, Ctia=16pF）
  6. 配置开关矩阵（D=OPEN, P=PL|PL2, N=OPEN, T=AIN1|TRTIA）
  7. 配置 ADC 基础（MuxP=HSTIA_P, MuxN=VSET1P1, PGA=1）
  8. 配置 ADC 滤波（SINC3 OSR=4, SINC2 OSR=22, ADCRate=1.6MHz）
  9. 使能：HPREFPWR|HSTIAPWR|INAMPPWR|EXTBUFPWR|DACREFPWR|SINC2NOTCH
 10. SEQ_STOP（序列只运行一次）
```

### SEQID_0（测量序列）详解

```
AppPHSeqMeasureGen() 生成的序列内容：
  0: ADCMux → HSTIA_P (P), VSET1P1 (N)
  1: AFECtrl: ADCPWR|HSTIAPWR|SINC2NOTCH|INAMPPWR|EXTBUFPWR = ON
  2: WAIT(16×80 clk)    ← ADC 稳定 ≈ 80μs
  3: AFECtrl: ADCCNV = ON   ← 开始转换
  4: WAIT(WaitClks)         ← 等待 SINC3 滤波完成
  5: AFECtrl: ADCCNV|ADCPWR = OFF  ← 停止并下电
  （无 EnterSleep，无 GPIO 标记，无扫频）
```

> **与电导率序列对比：**
> - 无 WG（不需要正弦波）
> - 无 DFT（直流测量）
> - 无 EnterSleep（AFE 在测完后不主动休眠）
> - 无唤醒定时器（ODR 由上层 main 循环控制）

---

## 数据处理算法

### PHShowResult（`ph_service.cpp:352`）

```
FIFO 数据格式（SINC3 模式）：
  每个 uint32_t 低 16 位为有效 ADC 码值

步骤：
1. rawCode = pData[i] & 0xFFFF          // 取低16位

2. diff_code = rawCode - ZeroOffset_Code // ZeroOffset_Code=32768（中点，待校准）

3. voltage_diff = (diff_code / 32768.0) × 1.82 V
                                         // ADC 参考电压 = 1.82V（内部基准）

4. current_A = -voltage_diff / Rtia_Value_Ohm
                                         // Rtia = 200Ω（当前配置）
                                         // 负号：HSTIA 输出 Vout = Vbias - I×Rtia
                                         //       ADC 测 (Vout - 1.1V) = -I×Rtia（近似）

5. current_uA = current_A × 10⁶         // 转换为微安打印
```

**示意换算（Rtia=200Ω，ZeroOffset=32768）：**
```
若 rawCode=32768（中点）→ diff=0 → V=0 → I=0 μA
若 rawCode=33768       → diff=1000 → V=55.6mV → I=-278 μA
```

### pH 换算（**尚未实现**，理论公式）

```
// 需补充的代码逻辑（Nernst 方程）：
float V_electrode = Vbias - current_A * Rtia;  // 电极电位
float pH = pH_ref + (V_electrode - E_ref) / (-0.05916f);  // 25°C，单位 V/pH
```

---

## ADS124S08 温度测量（PT1000）

### 驱动架构

```
ADS124S08_Drv（C++类）
  ├─ HAL 回调结构体（6个函数指针）
  │    cs_assert / cs_release / spi_txrx / delay_us / delay_ms / read_drdy
  ├─ SPI 命令：RESET/START/STOP/RDATA/RREG/WREG
  ├─ 寄存器读写：readRegisters / writeRegisters / writeRegisterMasked
  └─ 数据读取：readDataRaw（24位原始） / readData（32位符号扩展）
```

### ADS124S08 寄存器配置（defaultConfig，`ads124s08_drv.cpp:114`）

| 寄存器 | 地址 | 写入值 | 配置内容 |
|--------|------|--------|---------|
| INPMUX | 0x02 | `0x12` | AINP=AIN1, AINN=AIN2（差分输入） |
| PGA    | 0x03 | `0x09` | PGA 使能，增益配置（TODO：按手册核验） |
| DATARATE | 0x04 | `0x14` | 连续转换，20 SPS，低延迟滤波 |
| REF    | 0x05 | `0x02` | 外部参考（TODO：具体位域待核验） |
| IDACMAG | 0x06 | `0x05` | IDAC 激励电流 = 500 μA |
| IDACMUX | 0x07 | `0xF0` | IDAC1→AIN0，IDAC2→NC（TODO：核验编码）|
| SYS    | 0x09 | `0x10` | 关闭 SENDSTAT/CRC |

### PT1000 温度计算（`readPT1000Temperature`，`ads124s08_drv.cpp:214`）

```
测量方法：比率法（Ratiometric）
  激励：IDAC = 500μA，施加到 AIN0
  测量：AIN1-AIN2 两端电压
  参考电压：外部参考（r_ref 即参比电阻）

公式：
  ratio  = ADC_code / 2^23          (2^23 = 8,388,608 = FS)
  R_PT1000 = ratio × R_ref / gain   (gain=2)

温度换算（Callendar-Van Dusen，0-850°C 范围，代码未实现）：
  R(T) = R0 × (1 + A×T + B×T²)
  A = 3.9083×10⁻³, B = -5.775×10⁻⁷
  R0 = 1000Ω（PT1000 在 0°C 的电阻）

  → 反解 T（用于 0°C 以上）：T = (-A + √(A² - 4B(1 - R/R0))) / (2B)
```

> **注意：`readPT1000Temperature` 函数体在范围检查后没有 return 语句和温度赋值**，属于未完成的实现。

---

## 与其他模块的接口

### 被调用者

| 模块 | 调用关系 |
|------|---------|
| `ad5941PlatformCfg.cpp` | 平台初始化（`AD5941PlatformCfg()`）必须先于 `AppPHInit()` 调用 |
| `ad5941_board_glue.cpp` | `Ad5941Glue::setup(cfg)` 绑定 SPI（共用 AD5940 驱动） |
| `ads124s08_board_glue.cpp` | `BoardGlue::make_ads_hal(cfg)` 为 ADS124S08 绑定独立 SPI |
| `spi_hal.h` | `SpiDevice` 对象，两个芯片各自有独立的 CS 引脚 |

### 对外提供的 API

| 函数 | 作用 |
|------|------|
| `AppPHCfg_init()` | 加载默认参数（Vbias/Vzero/RTIA 等）|
| `AppPHGetCfg(void**)` | 获取配置结构体指针 |
| `AppPHInit(buf, size)` | 完整初始化：生成序列 + 加载 + 等待就绪 |
| `AppPHISR(buf, count)` | **轮询调用**，读 FIFO 原始数据 |
| `AppPHCtrl(cmd, para)` | 控制命令：START / STOPNOW / GETBIASVOLT / GETZEROVOLT / SHUTDOWN |
| `PHShowResult(data, n)` | 打印电流值（无 pH 换算） |

### 可配置参数

| 参数 | 默认值 | 影响 |
|------|--------|------|
| `DacData12Bit` | 0x745（≈1.135V）| 工作电极偏置电压 |
| `DacData6Bit` | 0x17（≈0.898V）| 参比/对电极零点 |
| `LpTiaRtia` | `LPTIARTIA_200R`（200Ω）| 电流量程（越小量程越大）|
| `ZeroOffset_Code` | 32768 | ADC 零点（需用标准 pH 液校准）|
| `Rtia_Value_Ohm` | 1000.0 Ω | 电流→电压转换系数（⚠️ 与上面 200Ω 不一致！）|

---

## 已知问题与注意事项

### Bug / 代码缺陷

| 编号 | 位置 | 问题描述 | 严重程度 |
|------|------|---------|---------|
| B1 | `PHShowResult:357` | `V_offset` 类型为 `uint32_t` 却赋了 `float`（1000.0f），且 `V_offset` 从未使用 | 低（无影响） |
| B2 | `PHShowResult:356,377` | `RTIA_VAL` 取 `Rtia_Value_Ohm=1000.0`，但实际配置是 200Ω（`LPTIARTIA_200R`）—— **计算电流会差 5 倍** | **高** |
| B3 | `AppPHCtrl:91-97` | `PHCTRL_GETBIASVOLT` case 缺少 `break`，**直接 fall-through** 到 `PHCTRL_GETZEROVOLT` | 中 |
| B4 | `AppPHCtrl:87-89` | `PHCTRL_STOPNOW` 只置标志位，没有先唤醒 AFE 再停止 WUPT（与电导率服务不一致） | 低 |
| B5 | `readPT1000Temperature:240-252` | 函数在范围检查后无 return 语句，**返回值未定义（UB）** | **高** |
| B6 | `defaultConfig` 中 | 多处 TODO 注释，部分寄存器值可能不正确（IDACMUX=0xF0 待核验）| 中 |

### 架构说明

1. **pH 未实现 Nernst 换算**：`PHShowResult` 只输出电流（μA），不输出 pH 值，需在上层或函数内补充
2. **无 WUPT 自动触发**：pH 模块没有配置唤醒定时器（ODR），需要 main 循环调用 `AppPHCtrl(PHCTRL_START)` 手动触发每次测量
3. **两套 RTIA 参数不同步**：`LpTiaRtia=LPTIARTIA_200R`（实际 200Ω）与 `Rtia_Value_Ohm=1000.0`（计算用 1kΩ）不一致
4. **无扫频支持**：pH 不需要扫频，这是合理设计
5. **ADS124S08 为独立 SPI 设备**：与 AD5940 使用不同的 CS 引脚，可以同时工作
