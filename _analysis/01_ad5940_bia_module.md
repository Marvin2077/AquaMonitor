# AD5940/AD5941 阻抗测量模块分析

> 分析日期：2026-02-22
> 涉及文件：
> - [src/ad5940.c](../src/ad5940.c) — ADI 官方驱动库（4422 行，纯 C）
> - [src/ad5941_board_glue.cpp](../src/ad5941_board_glue.cpp) — ESP32 平台适配层
> - [src/ad5941PlatformCfg.cpp](../src/ad5941PlatformCfg.cpp) — 平台级初始化
> - [src/Conductivity_service.cpp](../src/Conductivity_service.cpp) — 电导率应用服务
> - [include/ad5940.h](../include/ad5940.h) — 驱动头文件（340KB，含全部寄存器定义）
> - [include/Conductivity_service.h](../include/Conductivity_service.h) — 应用层接口

---

## 模块概述

AD5940/AD5941 是 ADI 公司的高精度**电化学/阻抗测量 AFE（模拟前端）芯片**，本项目用于测量**水溶液电导率**。

### 功能分层

```
┌─────────────────────────────────────────────────────┐
│           应用层 Conductivity_service.cpp            │
│   AppCondInit / AppCondISR / AppCondCtrl             │
│   CondShowResult / ComputeKCell                      │
├─────────────────────────────────────────────────────┤
│          平台配置 ad5941PlatformCfg.cpp               │
│   AD5941PlatformCfg(): 时钟/FIFO/中断/GPIO           │
├─────────────────────────────────────────────────────┤
│          ADI 驱动库 ad5940.c（约100个函数）           │
│   寄存器操作、序列发生器、校准、复数运算               │
├─────────────────────────────────────────────────────┤
│          板级胶合层 ad5941_board_glue.cpp             │
│   SPI CS/读写/延时/复位 → ESP32 Arduino SPI          │
└─────────────────────────────────────────────────────┘
```

### 测量原理

采用 **两端交流阻抗法**：
1. 向传感器施加已知频率的正弦激励电压（HSDAC + 波形发生器）
2. 用 HSTIA（跨阻放大器）将传感器电流转换为电压
3. ADC + DFT 分别测量 **V_电流（HSTIA两端）** 和 **V_电压（传感器电极）**
4. 复数除法计算阻抗 Z = V / I，再换算为电导率

---

## 核心寄存器配置

### 时钟寄存器组（AFECON 域）

| 寄存器 | 地址 | 本项目配置 | 说明 |
|--------|------|-----------|------|
| `CLKCON0` | `0x0408` | SYSCLKDIV=1, ADCCLKDIV=1 | 时钟分频配置 |
| `CLKSEL`  | `0x0414` | SYSCLKSRC=HFOSC, ADCCLKSRC=HFOSC | 时钟源选择 |
| `CLKEN1`  | `0x0410` | 默认 | 时钟门控 |

**配置值（`ad5941PlatformCfg.cpp:18-26`）：**
```c
clk_cfg.SysClkSrc     = SYSCLKSRC_HFOSC;   // 高频振荡器
clk_cfg.ADCCLkSrc     = ADCCLKSRC_HFOSC;
clk_cfg.SysClkDiv     = SYSCLKDIV_1;        // 不分频
clk_cfg.ADCClkDiv     = ADCCLKDIV_1;
clk_cfg.HfOSC32MHzMode = bTRUE;            // 使能 32MHz 模式（低频时降为 16MHz）
clk_cfg.HFOSCEn      = bTRUE;
clk_cfg.LFOSCEn      = bTRUE;              // 32kHz LFOSC 用于唤醒定时器
```

> 实际工作频率：低频激励（<20kHz）时用 16MHz；高频激励（≥20kHz）时切换到 32MHz 高功耗模式（`AD5940_HPModeEn(bTRUE)`）。

---

### AFE 控制寄存器（`REG_AFE_AFECON` = `0x2000`）

| 位 | 位名 | 使能条件 | 说明 |
|----|------|---------|------|
| [21] | DACBUFEN | 初始化时 | DC DAC 缓冲器 |
| [20] | DACREFEN | 初始化时 | HSDAC 参考使能 |
| [16] | SINC2EN  | 初始化时 | ADC SINC2/Notch 滤波器 |
| [15] | DFTEN    | 测量时   | DFT 硬件加速器 |
| [14] | WAVEGENEN| 测量时   | 波形发生器 |
| [11] | TIAEN    | 初始化时 | 高功耗 TIA |
| [10] | INAMPEN  | 初始化时 | 激励放大器 |
| [9]  | EXBUFEN  | 初始化时 | 激励缓冲器 |
| [8]  | ADCCONVEN| 测量时   | ADC 转换启动 |
| [7]  | ADCEN    | 测量时   | ADC 电源 |
| [6]  | DACEN    | 初始化时 | HSDAC 使能 |

---

### 激励 DAC 配置（`REG_AFE_HSDACCON` = `0x2010`）

| 字段 | 本项目值 | 说明 |
|------|---------|------|
| `INAMPGNMDE` | `EXCITBUFGAIN_2`（×2） | 激励放大器增益，自动根据 Rcal/Rtia 比值选择 |
| `ATTENEN`    | `HSDACGAIN_1`（×1）    | DAC PGA 衰减，自动选择 |
| `RATE`       | 7（低频）/ 7（高频）   | DAC 更新速率 = SysClk / Rate |

**激励电压计算（`ad5940.c:3458`）：**
```c
ExcitVolt = 1800 * 0.8 * Rcal / Rtia;  // 单位 mVpp
// 根据结果自动选择 ExcitBuffGain 和 HsDacGain
```

**用户配置（`Conductivity_service.cpp:28`）：**
```c
AppCondCfg.DacVoltPP = 600.0;          // 600 mVpp 激励
AppCondCfg.SinFreq   = 50000.0;        // 50 kHz
AppCondCfg.ExcitBufGain = EXCITBUFGAIN_2;
AppCondCfg.HsDacGain    = HSDACGAIN_1;
```

---

### HSTIA 跨阻放大器（`REG_AFE_HSTIACON` = `0x20FC`）

| 参数 | 配置值 | 说明 |
|------|--------|------|
| `HstiaRtiaSel` | `HSTIARTIA_1K`（1kΩ） | 内部反馈电阻，量程决定因素 |
| `HstiaCtia`    | 16 pF | 并联电容，抑制高频噪声 |
| `HstiaBias`    | `HSTIABIAS_1P1`（1.1V） | TIA 偏置电压 |
| `HstiaDeRload` | `HSTIADERLOAD_OPEN` | DE0 负载开路 |
| `HstiaDeRtia`  | `HSTIADERTIA_OPEN`  | DE0 RTIA 开路 |

> RTIA 可选范围：200Ω / 1kΩ / 5kΩ / 10kΩ / 20kΩ / 40kΩ / 80kΩ / 160kΩ

---

### DFT 配置（`REG_AFE_DFTCON` = `0x20D0`）

| 参数 | 50kHz 时配置 | 说明 |
|------|------------|------|
| `DftNum`  | `DFTNUM_8192`（8192点） | DFT 点数 = 2^(N+2) |
| `DftSrc`  | `DFTSRC_SINC3`          | DFT 输入来自 SINC3 滤波器 |
| `HanWinEn`| `bTRUE`                 | 汉宁窗，减少频谱泄漏 |

**频率自适应规则（`AD5940_GetFreqParameters`，`ad5940.c:780`）：**

| 频率范围 | DFT 源 | 时钟模式 | SINC3 OSR | SINC2 OSR |
|---------|--------|---------|-----------|-----------|
| ≥ 20kHz | SINC3  | 高功耗（32MHz）| 2 | — |
| < 20kHz | SINC2+Notch | 低功耗（16MHz）| 4 | 自动计算 |
| < 0.51Hz| SINC2+Notch | 高功耗 | 1 | OSR=533 |

---

### ADC 滤波链配置

```
ADC(800KHz/1.6MHz) → SINC3(OSR=2/4/5) → SINC2+Notch(OSR=22..1333) → DFT(8192点)
```

| 参数 | 低速测量配置 |
|------|------------|
| `ADCRate`    | `ADCRATE_800KHZ`（16MHz ADC 时钟时） |
| `ADCSinc3Osr`| `ADCSINC3OSR_2` |
| `ADCSinc2Osr`| `ADCSINC2OSR_22` |
| `ADCPgaGain` | `ADCPGA_1P5`（×1.5） |
| `BpNotch`    | `bTRUE`（旁路 Notch）|
| `Sinc2NotchEnable` | `bTRUE` |

---

### FIFO 配置（`ad5941PlatformCfg.cpp:29-37`）

| 参数 | 值 | 说明 |
|------|---|------|
| `FIFOSize`   | 4KB | 总 SRAM 6KB，4KB 给 FIFO，2KB 给序列器 |
| `FIFOSrc`    | `FIFOSRC_DFT` | DFT 结果直接入 FIFO |
| `FIFOThresh` | 4 | 每次测量产生 4 个 uint32_t（2 个复数 DFT 结果）|

---

## 测量流程

### 总体流程图

```
上电/复位
    │
    ▼
AD5941PlatformCfg()
  ├─ HWReset + Initialize（写入魔法寄存器序列）
  ├─ 配置时钟（HFOSC 16/32MHz）
  ├─ 配置 FIFO（4KB，FIFOSRC_DFT，阈值=4）
  ├─ 配置 INTC1=ALL，INTC0=DATAFIFOTHRESH
  └─ 配置 GPIO（GP0=INT，GP1=SLEEP，GP2=SYNC）
    │
    ▼
AppCondCfg_init()
  └─ 加载默认参数（50kHz，600mVpp，1kΩ RTIA，ODR=20Hz）
    │
    ▼
AppCondInit(buffer, bufSize)
  ├─ 唤醒 AFE（轮询 ADIID 寄存器）
  ├─ 配置序列器（2KB SRAM）
  ├─ [首次/ReDoRtiaCal] AppCondRtiaCal()──────────┐
  │    └─ AD5940_HSRtiaCal() × N（每频点一次）    │ RTIA 校准
  │         扫频时建立 RtiaCalTable[100]           │
  ├─ 重新配置 FIFO（使能）                         │
  ├─ AppCondSeqCfgGen()  → 写入 SEQID_1（初始化序列）
  ├─ AppCondSeqMeasureGen() → 写入 SEQID_0（测量序列）
  ├─ 触发 SEQID_1 运行，等待 ENDSEQ 中断
  ├─ 配置 SEQID_0，设置唤醒定时器（ODR=20Hz）
  └─ AFE 进入睡眠，等待 WUPT 触发
    │
    ▼（每 1/ODR = 50ms 触发一次）
  ┌─────────────────────────────────┐
  │     SEQID_0 自动执行序列         │  ← 唤醒定时器触发
  │  1. 切换开关矩阵接通传感器        │
  │  2. 等待 16×250 clk 稳定         │
  │  3. ADCMux → HSTIA_P/N           │  测电流
  │     WG+ADC ON → ADC+DFT 启动    │
  │     等待 DFT 完成 → 关闭         │
  │  4. ADCMux → VCE0/N_NODE         │  测电压
  │     WG+ADC ON → ADC+DFT 启动    │
  │     等待 DFT 完成 → 关闭         │
  │  5. 开关矩阵浮空 → AFE 进入休眠  │
  └────────────┬────────────────────┘
               │ FIFO 达到阈值（4条）
               ▼
        AppCondISR(buffer, count)
          ├─ 唤醒 AFE
          ├─ 读取 FIFO（4 × uint32_t）
          ├─ AppCondRegModify()（扫频时更新下一频点）
          ├─ AFE 再次进入休眠
          └─ AppCondDataProcess() → 输出阻抗复数
               │
               ▼
        CondShowResult() / ComputeKCell()
          └─ 计算电导率并输出（Serial.printf）
```

---

### SEQID_0 测量序列详解

**序列存储位置：** AD5940 内部 SRAM（在 InitSeq 之后的地址）

```
序列命令（由 AppCondSeqMeasureGen 生成）：
 0: GPIO_CTRL(AGPIO_Pin1=HIGH)  ← 标记测量开始
 1: SWMatrix → D=CE0, P=CE0, N=AIN0, T=AIN0|TRTIA  ← 接通传感器
 2: WAIT(16*250)                ← 等待约 250μs（16MHz 时钟）

 === 步骤1：测量电流（I = Vhstia / Rtia）===
 3: ADCMux = HSTIA_P (P), HSTIA_N (N)
 4: AFECtrl: WG=ON, ADC_PWR=ON
 5: WAIT(16*80)                 ← ADC 稳定延时
 6: AFECtrl: ADC_CONV=ON, DFT=ON
 7: WAIT(WaitClks)              ← 等待 DFT 完成（根据频率动态更新）
 8: WAIT(1)
 9: AFECtrl: ADC_CONV=OFF, DFT=OFF, WG=OFF, ADC_PWR=OFF

 === 步骤2：测量电压（V = Vce0）===
10: ADCMux = VCE0 (P), N_NODE (N)
11: AFECtrl: WG=ON, ADC_PWR=ON
12: WAIT(16*80)
13: AFECtrl: ADC_CONV=ON, DFT=ON
14: WAIT(WaitClks)              ← 动态更新位置: SRAMAddr+18
15: WAIT(1)
16: AFECtrl: ADC_CONV=OFF, DFT=OFF, WG=OFF, ADC_PWR=OFF

17: SWMatrix → D=OPEN, P=PL|PL2, N=NL|NL2, T=TRTIA  ← 断开传感器
18: GPIO_CTRL(0)                ← 测量结束标记
19: EnterSleep                  ← AFE 进入休眠
```

> **动态更新：** `AppCondCheckFreq()` 在扫频时会直接修改 SRAM 中的 WaitClks 命令字，无需重新生成整个序列。

---

## 数据处理算法

### 1. FIFO 数据格式

每次 ISR 读出 4 个 `uint32_t`，排布如下：

```
pData[0] = DFT_I_Real   (18位补码，bit[17]=符号位)
pData[1] = DFT_I_Imag
pData[2] = DFT_V_Real
pData[3] = DFT_V_Imag
```

### 2. 符号扩展（`AppCondDataProcess`，`Conductivity_service.cpp:610-617`）

```c
pData[i] &= 0x3ffff;                // 保留低 18 位
if(pData[i] & (1<<17))              // bit17 是符号位
    pData[i] |= 0xfffc0000;         // 符号扩展到 32 位有符号
```

### 3. 阻抗计算

```
注意：ADI 芯片 DFT 虚部寄存器值是真实虚部的相反数
     电流方向为 HSTIA_N → HSTIA_P，所以实部也要取反

DftCurr.Real  = -pData[0]    (取反修正方向)
DftCurr.Image = -pData[1]

DftVolt.Real  = pData[2]
DftVolt.Image = pData[3]

I = DftCurr / RtiaCurrValue          (复数除法，得到校准后电流)
Z = DftVolt / I                       (复数除法，得到阻抗)

Z = (DftVolt × RtiaCurrValue) / DftCurr
```

**代码（`Conductivity_service.cpp:632-634`）：**
```c
res = AD5940_ComplexDivFloat(&DftCurr, &AppCondCfg.RtiaCurrValue);  // I = V_rtia / Z_rtia
res = AD5940_ComplexDivFloat(&DftVolt, &res);                        // Z = V_sensor / I
```

### 4. 电导率计算（`CondShowResult`，`Conductivity_service.cpp:709-750`）

```
R = Z.Real       (阻抗实部，单位 Ω)
X = Z.Image      (阻抗虚部，单位 Ω)

|Z|² = R² + X²

G = R / |Z|²                   (电导，单位 S)
G_uS = G × 10⁶                 (微西门子)

Conductivity = G_uS × K_Cell   (电导率，单位 μS/cm)
```

其中 `K_Cell` 为电极常数（单位 cm⁻¹，默认 1.0），由实际电极几何形状决定。

### 5. RTIA 校准算法（`AD5940_HSRtiaCal`，`ad5940.c:3396`）

```
校准原理：接入已知精度电阻 Rcal，实测 RTIA 的复阻抗

步骤：
1. 开关矩阵连接 Rcal（D=RCAL0, P=RCAL0, N=RCAL1, T=RCAL1|TRTIA|AIN1）
2. 施加激励电压，ADCMux = P_NODE/N_NODE，DFT → DftRcalVolt（Rcal 两端电压）
3. ADCMux = HSTIA_P/HSTIA_N，DFT → DftRtiaVolt（RTIA 两端电压）
4. 符号修正（虚部取反 + 方向修正）
5. Rtia = (DftRtiaVolt / DftRcalVolt) × Rcal
```

**激励电压自动档位选择：**

| 条件 | ExcitBufGain | HsDacGain | 满量程电压 |
|------|-------------|-----------|-----------|
| ExcitVolt ≤ 40mVpp | ×0.25 | ×0.2 | 40mVpp |
| ExcitVolt ≤ 200mVpp | ×0.25 | ×1 | 200mVpp |
| ExcitVolt ≤ 320mVpp | ×2 | ×0.2 | 320mVpp |
| ExcitVolt > 320mVpp | ×2 | ×1 | 1600mVpp |

---

## 与其他模块的接口

### 被调用者（Conductivity_service 依赖）

| 函数 / 模块 | 接口说明 |
|------------|---------|
| `ad5941PlatformCfg.cpp` | `AD5941PlatformCfg()` — 必须在 AppCondInit 之前调用 |
| `ad5941_board_glue.cpp` | `Ad5941Glue::setup(cfg)` — 绑定 SPI 设备，必须最先调用 |
| `spi_hal.h` | `SpiDevice` — SPI 设备抽象，glue 层通过它收发数据 |

### 对外提供的 API（供 main.cpp / host_link 调用）

| 函数 | 作用 |
|------|------|
| `AppCondCfg_init()` | 加载默认配置参数 |
| `AppCondGetCfg(void** pCfg)` | 获取配置结构体指针，供外部修改参数 |
| `AppCondInit(buffer, size)` | 完整初始化：序列生成 + RTIA 校准 + 启动 |
| `AppCondISR(buffer, count)` | **轮询调用**（非中断），读 FIFO 并返回阻抗数据 |
| `AppCondCtrl(cmd, para)` | 控制命令：START / STOPNOW / STOPSYNC / SHUTDOWN / GETFREQ |
| `CondShowResult(data, count)` | 打印阻抗和电导率到 Serial |
| `ComputeKCell(data, count)` | 只计算并返回电导值（μS） |

### 可配置参数（通过 `AppCondCfg` 全局结构体）

| 参数 | 默认值 | 影响 |
|------|--------|------|
| `SinFreq` | 50000 Hz | 激励频率 |
| `DacVoltPP` | 600 mVpp | 激励幅度 |
| `HstiaRtiaSel` | HSTIARTIA_1K | 量程（电流检测范围） |
| `CondODR` | 20 Hz | 输出数据率（唤醒定时器周期） |
| `RcalVal` | 1000 Ω | 校准电阻值 |
| `K_Cell` | 1.0 cm⁻¹ | 电极常数 |
| `SweepCfg.SweepEn` | bFALSE | 扫频开关（EIS 谱测量） |
| `SweepCfg.SweepStart/Stop` | 2kHz–200kHz | 扫频范围 |
| `SweepCfg.SweepPoints` | 100 | 扫频点数 |
| `DftNum` | DFTNUM_8192 | 频率分辨率 |
| `HanWinEn` | bTRUE | 汉宁窗减少泄漏 |

---

## 中断处理说明

> **本项目使用轮询而非硬件中断！**

```cpp
// ad5941_board_glue.cpp:71-84
extern "C" uint32_t AD5940_GetMCUIntFlag(void) { return 0; }  // 永远返回无中断
extern "C" uint32_t AD5940_ClrMCUIntFlag(void) { return 1; }  // 空操作
```

**轮询调用方式：**
```
main loop 或 host_link 周期性调用 AppCondISR()
    → 内部检查 INTC0 的 DATAFIFOTHRESH 标志位
    → 若置位则读取 FIFO 并返回数据
    → 否则直接返回
```

**INTC 架构：**
- `INTC1`（内部监控）：使能全部中断源，用于轮询查询状态
- `INTC0`（外部输出）：仅使能 `DATAFIFOTHRESH`，触发 GP0 引脚（未使用）

---

## 已知限制与注意事项

1. **序列 SRAM 紧张**：仅 2KB 给序列器，初始化 + 测量序列共享，测量序列起始地址 = `InitSeq.addr + InitSeq.len`
2. **WaitClks 上限**：最大 `0x3FFFFFFF`，超出时需将等待拆成两个连续 WAIT 命令（代码 `Conductivity_service.cpp:550-566`）
3. **扫频校准时间长**：扫频 100 个频点时，需对每个频点分别执行 RTIA 校准（100次×校准时间）
4. **`freecl_service.cpp` 为空**：游离氯服务尚未实现（0 行）
5. **K_Cell 默认为 1.0**：实际使用时需要用已知溶液标定电极常数
6. **50kHz 固定单频**：默认不开扫频（`SweepEn = bFALSE`），EIS 谱测量需手动开启
7. **电流方向符号修正**：DFT 虚部寄存器值是真实虚部的相反数（ADI 芯片特性），代码中有两次取反抵消，最终结果正确
