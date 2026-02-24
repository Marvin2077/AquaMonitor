# 03 TMUX 模拟多路复用器模块分析

**分析文件**: `include/mux_iface.h`, `src/mux_iface.cpp`, `src/main.cpp`
**分析日期**: 2026-02-22

---

## 多路复用器配置

代码中实现了**两个独立的模拟多路复用器**，均通过 GPIO 地址线控制。

### MUX-A：传感通道选择器（Channel MUX）

| 属性 | 值 |
|------|-----|
| 型号推断 | 双通道差分 MUX（类 TMUX1134 / TMUX109x 系列） |
| 地址位宽 | 2-bit（ADDR1, ADDR0） |
| 有效通道数 | 3（S1A/S1B、S2A/S2B、S3A/S3B） |
| 最大通道数 | 4（2-bit 可寻址） |
| 控制 GPIO | IO17（ADDR0）、IO18（ADDR1） |
| 输出形式 | 差分双路（A/B 配对同步切换） |

> **型号说明**：代码中未出现 1108/1109/1134 等具体型号字符串，需查阅原理图或 BOM。
> 输出采用 A/B 配对（S1A & S1B 同时切换），与 TMUX1134（4通道双刀）行为一致。

### MUX-B：ISFET 通道选择器（ISFET MUX）

| 属性 | 值 |
|------|-----|
| 型号推断 | 单端 8:1 MUX（类 TMUX1108 系列） |
| 地址位宽 | 3-bit（ADDR2, ADDR1, ADDR0） |
| 有效通道数 | 8（S1～S8） |
| 控制 GPIO | IO32（ADDR0）、IO33（ADDR1）、IO25（ADDR2） |
| 输出形式 | 单端 |

---

## 通道映射关系

### MUX-A 传感通道映射

| 通道号 | ADDR1 (IO18) | ADDR0 (IO17) | 选中端口 | 对应传感器/功能 |
|--------|-------------|-------------|---------|---------------|
| CH1 | LOW (0) | LOW (0) | S1A & S1B | 阻抗 / 电导率（Impedance） |
| CH2 | LOW (0) | HIGH (1) | S2A & S2B | 余氯（Residual Chlorine） |
| CH3 | HIGH (1) | LOW (0) | S3A & S3B | pH 值 |
| CH4 | HIGH (1) | HIGH (1) | S4A & S4B | 未使用（保留） |

**初始化默认通道**：`MUXPinInit()` 完成后立即调用 `ChooseSenesingChannel(1)` → CH1（阻抗）。

### MUX-B ISFET 通道映射（8 选 1）

| 通道 | ADDR2 (IO25) | ADDR1 (IO33) | ADDR0 (IO32) | 选中端口 |
|------|-------------|-------------|-------------|---------|
| CH1 | 0 | 0 | 0 | S1 |
| CH2 | 0 | 0 | 1 | S2 |
| CH3 | 0 | 1 | 0 | S3 |
| CH4 | 0 | 1 | 1 | S4 |
| CH5 | 1 | 0 | 0 | S5 |
| CH6 | 1 | 0 | 1 | S6 |
| CH7 | 1 | 1 | 0 | S7 |
| CH8 | 1 | 1 | 1 | S8 |

**初始化默认通道**：`MUXPinInit()` 将 ISFET MUX 置于 CH1（全零）。

---

## 切换控制接口

### API 定义（`include/mux_iface.h`）

```c
void MUXPinInit(void);                    // 初始化所有 MUX GPIO，置默认通道
void ChooseSenesingChannel(int channel);  // 切换传感通道（1~3）
void ChooseISFETChannel(int channel);     // 切换 ISFET 通道（1~8）
```

### 控制机制

- **纯 GPIO 驱动**：通过 `digitalWrite()` 直接操作地址引脚，无 SPI/I2C 通信开销
- **无显式建立时间延迟**：`mux_iface.cpp` 中切换后**不插入任何 delay()**，依赖 GPIO 翻转速率自然建立
- **default 分支异常**：`ChooseSenesingChannel()` 的 `default` 分支存在 `digitalWrite(17|18, LOW)` 写法错误（`17|18 = 27`，非有效引脚），实际应分两次写

### 初始化序列（`MUXPinInit()`）

```
CHANNEL_MUX_ADDR1 (IO18) = HIGH   ← 注意：初始为HIGH
CHANNEL_MUX_ADDR0 (IO17) = LOW
→ 对应 CH3（pH），但紧随其后被 ChooseSenesingChannel(1) 覆盖为 CH1
ISFET_MUX_ADDR2 (IO25) = LOW
ISFET_MUX_ADDR1 (IO33) = LOW
ISFET_MUX_ADDR0 (IO32) = LOW
→ 对应 CH1（S1）
```

---

## 典型切换序列

### 序列 1：系统启动初始化

```
setup() 开始
  └─ delay(1000)                   // 等待串口和设备ID就绪
  └─ MUXPinInit()                  // GPIO 初始化
  └─ ChooseSenesingChannel(1)      // → CH1 阻抗通道（ADDR1=0, ADDR0=0）
  └─ ChooseISFETChannel(1)         // → ISFET CH1（全地址为0）
  └─ SpiHAL::beginBus(...)         // 初始化 SPI 总线
  └─ AD5940 初始化 ...
```

### 序列 2：启动电导率测量

```
CMD: "cond measure"
  └─ ChooseSenesingChannel(1)      // → CH1 阻抗/电导通道
  └─ AppCondCtrl(CondCTRL_START)   // AD5940 启动阻抗测量
  └─ currentState = STATE_COND_MEASURE

loop() → STATE_COND_MEASURE
  └─ AppCondISR(AppBuff, &count)   // 等待 AD5940 中断
  └─ CondShowResult(...)           // 输出结果
```

### 序列 3：切换至 pH 测量

```
CMD: "ph init"
  └─ ChooseSenesingChannel(2)      // ⚠ 疑似Bug：切至余氯通道(CH2)
  └─ currentState = STATE_PH_INIT

loop() → STATE_PH_INIT
  └─ ChooseSenesingChannel(3)      // 修正为 CH3（pH通道）
  └─ AppPHInit(AppBuff, SIZE)      // 初始化 AD5940 pH 测量模式
  └─ currentState = STATE_IDLE

CMD: "ph read"
  └─ ChooseSenesingChannel(2)      // ⚠ 疑似Bug：再次切至余氯通道(CH2)
  └─ currentState = STATE_PH_MEASURE
  └─ AppPHCtrl(PHCTRL_START)       // AD5940 启动 pH 采集（通道不确定）
```

> **⚠ 代码一致性问题**：
> `ChooseSenesingChannel` 注释明确 CH2 = 余氯、CH3 = pH，但 "ph init" / "ph read" /
> "ph cal offset" / "ph cal gain" 等 pH 相关命令均调用 CH2（余氯通道），而 `STATE_PH_INIT`
> 状态内部调用的是 CH3（pH 通道）。两者不一致，可能造成 pH 测量实际连接余氯传感器路径。
> 建议核查原理图确认 MUX 硬件实际接线。

### 序列 4：ISFET 通道轮询（示例）

```
for channel = 1 to 8:
  ChooseISFETChannel(channel)      // GPIO 直写，无延迟
  [此处应触发 ADC 采样，代码中暂未找到调用点]
```

---

## 与测量模块的配合关系

```
┌─────────────────────────────────────────────────────────┐
│                      main.cpp 状态机                     │
│                                                          │
│  CMD ──→ ChooseSenesingChannel(N)  ← 先切换MUX           │
│                    │                                     │
│                    ▼                                     │
│          AppCondInit / AppPHInit   ← 再初始化AD5940       │
│                    │                                     │
│                    ▼                                     │
│          AppCondCtrl / AppPHCtrl   ← 启动采集             │
│                    │                                     │
│                    ▼                                     │
│          AppCondISR / AppPHISR     ← 等待完成             │
│                    │                                     │
│                    ▼                                     │
│          CondShowResult / PHShowResult                   │
└─────────────────────────────────────────────────────────┘

信号路径：
  传感器 → [MUX-A 传感通道] → AD5940 模拟输入
  ISFET  → [MUX-B ISFET通道] → [ISFET 前端电路] → AD5940 / ADC
```

**关键约束**：
1. MUX 切换**先于** AD5940 初始化/启动执行
2. 无专用稳定等待时间，MUX 建立时间依赖 GPIO 驱动速率（ESP32 约 ns 级）
3. 测量期间不进行通道切换（状态机保证互斥）
4. `g_isCondMode` / `g_ispHMode` 标志与 MUX 通道状态需保持同步

---

*分析基于源代码，TMUX 具体型号需结合硬件原理图确认。*
