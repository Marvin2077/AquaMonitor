#include <Arduino.h>
#include <SPI.h>

//所有驱动层头文件
#include "spi_hal.h"        // 底层硬件操作
#include "ads124s08_board_glue.h"     // 胶水层
#include "ads124s08_drv.h"  // 通用驱动层
#include "ad5941_board_glue.h"
#include "ad5941_drv.h"
#include "cond_service.h"
#include "temp_service.h" 
#include "storage_manager.h"


/* ====== 共用 SPI 线 ====== */
static const int PIN_SCLK = 14;
static const int PIN_MISO = 12;
static const int PIN_MOSI = 13;

/* ====== 片选/辅助 ====== */
// ADS124S08
static const int CS_ADS    = 20;
static const int DRDY_ADS  = 4; // 数据手册中为 GPIO4，请根据您的板子确认
static const int RESET_ADS = 21;
// ====== AD5941 片选/辅助 ======
static const int CS_AD5941 = 26; // 假设 AD5941 的 CS 连接到 GPIO 26
static const int RESET_AD5941 = 25; // 假设 RESET 连接到 GPIO 25 (如果连接了)
// static const int INT_AD5941 = 34;   // 假设中断输出连接到 GPIO 34 (仅输入引脚)

// 定义外部参考电压值，用于后续电压转换计算 请根据您实际连接到 REFP0/REFN0 的参考电压修改此值
const double V_REF = 1.68; 

// 创建一个 ADS124S08 驱动对象（此时还未初始化）
ADS124S08_Drv* adc = nullptr;
// 创建 AD5941 驱动对象指针
AD5941_Drv* afe = nullptr;
//创建 Temp Service 对象指针
TempService* g_tempSvc = nullptr;
// —— 电导率测试服务 —— 
static CondService*   g_condSvc = nullptr;
static CondConfig     g_condCfg;   

// === Calibration control ===
static bool cal_enabled = false;   // 默认不进校准流程
// 可选：启动就想进校准可改为 true
extern void handleCondCommand(const String& line); // 前置声明


void setup() {
  Serial.begin(115200);
  delay(1000);


  Serial.println("--- ADS124S08 PT1000 Demo ---");
  Serial.printf("Using theoretical V_REF = %.2fV\n", V_REF);
  SpiHAL::beginBus(PIN_SCLK, PIN_MISO, PIN_MOSI);
  Serial.println("SPI bus initialized.");
  
   // --- 初始化 ADS124S08 --- //
    Serial.println("Initializing ADS124S08...");
  static SpiDevice ads_spi_dev(CS_ADS, 4000000, MSBFIRST, SPI_MODE1);
  // 定义 ADS124S08 的 Glue 配置
  AdsGlueConfig ads_cfg = {
    .spi = &ads_spi_dev,
    .pin_drdy = DRDY_ADS,
    .pin_reset = RESET_ADS
  };
    Serial.println("Making ADS HAL...");
  ADS124S08_Drv::Hal ads_hal = BoardGlue::make_ads_hal(ads_cfg);
    // 创建ADS124S08驱动实例
  adc = new ADS124S08_Drv(ads_hal);
  Serial.println("ADS Driver instance created.");
  Serial.println("Configuring ADS124S08...");
  if (!adc->defaultConfig()) {
    Serial.println("!!! ADS124S08 configuration FAILED! Halting. !!!");
    while (1) { delay(1000); }
  }
  Serial.println("ADS124S08 configuration successful.");

  Serial.println("ADC started. Waiting for data...");
  Serial.println("----------------------------------------");

  // --- 完成ADS124S08的初始化 --- //

  // --- 初始化 AD5941 --- //
      Serial.println("Initializing AD5941...");
    static SpiDevice ad5941_spi_dev(CS_AD5941, 4000000, MSBFIRST, SPI_MODE0);
  // 定义 AD5941 的 Glue 配置
    Ad5941GlueConfig ad5941_cfg = {
        .spi = &ad5941_spi_dev,
        // .pin_reset = RESET_AD5941, // 如果使用硬件复位
        // .pin_interrupt = INT_AD5941 // 如果使用中断
    };
  // 创建 AD5941 的 HAL
    AD5941_Drv::Hal ad5941_hal = Ad5941BoardGlue::make_ad5941_hal(ad5941_cfg);
  // 创建 AD5941 驱动实例
  afe = new AD5941_Drv(ad5941_hal);

  // 执行初始化 (包括强制序列)
  
    if (!afe->begin()) {
        Serial.println("!!! AD5941 initialization FAILED! Halting. !!!");
        while (1) { delay(1000); }
    }
    Serial.println("AD5941 initialized successfully.");

  // 读取并打印芯片 ID 验证
  uint16_t adi_id, chip_id;
  if (afe->readChipID(adi_id, chip_id)) {
        Serial.printf("  ADI ID: 0x%04X, CHIP ID: 0x%04X\n", adi_id, chip_id);
   } else {
        Serial.println("  Failed to read Chip ID!");
   }
  // --- 完成AD5941 --- //

  // ==== 初始化 电导率测试服务 ====
  static CondService condSvc(*afe);   // 绑定已创建的 AD5941 驱动实例
  g_condSvc = &condSvc;

  // 根据需要可调整默认频率/幅值/DFT长度/带宽
  // 例：低频<80kHz用 normal 模式；≥80kHz 置 high_bw=true（PMBW.SYSHS=1）:contentReference[oaicite:3]{index=3}
  g_condCfg.freq_hz  = 10000.0f;      // 10 kHz
  g_condCfg.amp_code = 300;           // 适中幅度（示波器观察可调大些）
  g_condCfg.dft_n    = 4096;
  g_condCfg.high_bw  = (g_condCfg.freq_hz >= 80000.0f);
  g_condCfg.cell_K   = 1.0;           // 先用已知电阻标定时设为1
  g_condCfg.Rcal_ohm = 1000.0;        // 例如 1kΩ 精密电阻

  if (g_condSvc->begin(g_condCfg)) {
    Serial.println("[COND] Conductivity service ready.");
    // 在 setup() 中, g_condSvc->begin(g_condCfg) 成功之后：
Serial.println("--- 验证 AD5941 配置 ---");
uint32_t reg_val;
uint16_t reg_val16;

// 检查 AFECON (关键使能位)
if (afe->readRegister(AD5941_Drv::Reg::AFECON, reg_val)) {
    Serial.printf("AFECON (0x2000): 0x%08X\n", reg_val);
    // 检查位: WAVEGENEN(14), DACEN(6), EXBUFEN(9), HSREFDIS(5 必须为 0) [cite: 846, 853]
    bool wg_en = (reg_val >> 14) & 1;
    bool dac_en = (reg_val >> 6) & 1;
    bool exbuf_en = (reg_val >> 9) & 1;
    bool hsref_dis = (reg_val >> 5) & 1;
    Serial.printf("  WAVEGENEN(14)=%d, DACEN(6)=%d, EXBUFEN(9)=%d, HSREFDIS(5)=%d\n", wg_en, dac_en, exbuf_en, hsref_dis);
    if (!wg_en || !dac_en || !exbuf_en || hsref_dis) {
        Serial.println("  !!! AFECON 中关键使能缺失或参考被禁用!");
    }
} else { Serial.println("读取 AFECON 失败"); }

// 检查 WGCON (波形类型)
if (afe->readRegister(AD5941_Drv::Reg::WGCON, reg_val)) {
    Serial.printf("WGCON (0x2014): 0x%08X\n", reg_val);
    uint8_t type_sel = (reg_val >> 1) & 0x3; // [cite: 2975]
    Serial.printf("  TYPESEL(2:1)=%d (应为 2 代表正弦波)\n", type_sel);
     if (type_sel != 2) {
        Serial.println("  !!! 波形类型不是正弦波!");
    }
} else { Serial.println("读取 WGCON 失败"); }

// 检查 WGAMPLITUDE (幅度)
if (afe->readRegister(AD5941_Drv::Reg::WGAMPLITUDE, reg_val)) {
    Serial.printf("WGAMPLITUDE (0x203C): 0x%08X\n", reg_val);
    uint16_t amp = reg_val & 0x7FF; // Bits 10:0 [cite: 3020]
    Serial.printf("  SINEAMPLITUDE(10:0)=%u (应为 %u)\n", amp, g_condCfg.amp_code);
     if (amp == 0) {
        Serial.println("  !!! 幅度为零!");
    }
} else { Serial.println("读取 WGAMPLITUDE 失败"); }

// 检查开关矩阵控制 (SWCON)
if (afe->readRegister(AD5941_Drv::Reg::SWCON, reg_val)) {
    Serial.printf("SWCON (0x200C): 0x%08X\n", reg_val);
    bool src_sel = (reg_val >> 16) & 1; // SWSOURCESEL 
    bool t9_con = (reg_val >> 17) & 1; // 
    bool t10_con = (reg_val >> 18) & 1; // [cite: 2403]
    Serial.printf("  SWSOURCESEL(16)=%d (应为 1), T9CON(17)=%d, T10CON(18)=%d\n", src_sel, t9_con, t10_con);
     if (!src_sel) {
        Serial.println("  !!! 开关源未设置为 FULLCON 寄存器!");
    }
} else { Serial.println("读取 SWCON 失败"); }

// 检查 DSWFULLCON (D -> CE0 连接)
if (afe->readRegister(AD5941_Drv::Reg::DSWFULLCON, reg_val)) {
    Serial.printf("DSWFULLCON (0x2150): 0x%08X\n", reg_val);
    bool d5_set = (reg_val >> 5) & 1; // D5 开关对应 CE0 [cite: 2417]
    Serial.printf("  D5(5)=%d (应为 1 以连接 D 到 CE0)\n", d5_set);
    if (!d5_set) {
        Serial.println("  !!! D5 开关 (D->CE0) 未设置!");
    }
} else { Serial.println("读取 DSWFULLCON 失败"); }

// 检查 DSWSTA (D 开关的实际状态)
if (afe->readRegister(AD5941_Drv::Reg::DSWSTA, reg_val)) {
    Serial.printf("DSWSTA (0x21B0): 0x%08X\n", reg_val);
    // 注意: DSWSTA 的位映射可能与 FULLCON 不同，需要查手册确认 D5 状态位的位置
    // 假设 D5 状态在 Bit 4 (需要根据 Figure 36 和 Table 88 确认，看起来确实是 Bit 4 对应 D5STA) [cite: 2469]
    bool d5_sta = (reg_val >> 5) & 1;  // D5 -> bit5
    Serial.printf("  D5STA(4)=%d (如果 D5 关闭则应为 1)\n", d5_sta);
    if (!d5_sta) {
        Serial.println("  !!! DSWSTA 指示 D5 开关是断开的!");
    }
} else { Serial.println("读取 DSWSTA 失败"); }

// 检查 TSWFULLCON (T9/T10 控制 - 根据分析很重要)
if (afe->readRegister(AD5941_Drv::Reg::TSWFULLCON, reg_val)) {
    Serial.printf("TSWFULLCON (0x215C): 0x%08X\n", reg_val);
    bool t9_set = (reg_val >> 8) & 1; // [cite: 2456]
    bool t10_set = (reg_val >> 9) & 1; // [cite: 2456]
    Serial.printf("  T9(8)=%d (应为 0), T10(9)=%d (应为 1)\n", t9_set, t10_set);
    // 如果需要，根据预期的 TSWFULLCON 状态添加逻辑检查
} else { Serial.println("读取 TSWFULLCON 失败"); }


// 检查 PMBW (功率模式带宽)
if (afe->readRegister(AD5941_Drv::Reg::PMBW, reg_val)) {
    Serial.printf("PMBW (0x22F0): 0x%08X\n", reg_val);
    bool syshs = reg_val & 1; // [cite: 864]
    Serial.printf("  SYSHS(0)=%d (应匹配 high_bw: %d)\n", syshs, g_condCfg.high_bw);
} else { Serial.println("读取 PMBW 失败"); }

// 检查时钟选择 (CLKSEL)
if (afe->readRegister16(AD5941_Drv::Reg::CLKSEL, reg_val16)) {
     Serial.printf("CLKSEL (0x0414): 0x%04X\n", reg_val16);
     // Bits 1:0 = SYSCLKSEL, Bits 3:2 = ADCCLKSEL (0 = HFOSC) [cite: 3517, 3518]
     if ((reg_val16 & 0xF) != 0) {
        Serial.println("  !!! 时钟源可能不是内部高速振荡器 (HFOSC)!");
     }
} else { Serial.println("读取 CLKSEL 失败"); }

Serial.println("--- 结束验证 ---");
  } else {
    Serial.println("[COND] init failed.");
  }

  Serial.println();
  Serial.println("---- Conductivity commands ----");
  Serial.println("cond init [freqHz] [ampCode] [dftN]   # 重新配置(可省参数)");
  Serial.println("cond rcal [R]                         # 用已知电阻R做一次标定");
  Serial.println("cond read [Kcell]                     # 读一次并输出 (可临时设置K=1/cm)");
  Serial.println("cond help                             # 打印帮助");
  Serial.println("-------------------------------------");
  Serial.println("Tip: 用示波器探头看 DE0/CE0/RE0，10kHz 正弦应可见；增大 ampCode 可增幅。");


    // 初始化温度校准程序
  TempService::Config tcfg;
  tcfg.Rref_ohm = V_REF / 0.0005;  // 1.57V / 500uA ≈ 3140 Ω
  tcfg.pga_gain = 2;      // 与 defaultConfig 的增益保持一致
  static TempService tempSvc(*adc, tcfg);
  g_tempSvc = &tempSvc;
     //初始化&加载校准
  StorageManager::begin();
  TempService::CalibCoeff loaded;
  if (StorageManager::loadTempCalib(loaded) && loaded.valid) {
  g_tempSvc->setCalib(loaded);
  Serial.printf("[Cal] Loaded coeff from NVS: a=%.6g, b=%.6g, c=%.6g\n", loaded.a, loaded.b, loaded.c);
  } else {
  Serial.println("[Cal] No valid coeff in NVS (or CRC mismatch).");
  }
  // 结束加载校准
  Serial.println("[Cmd] Type: 'cal on'  -> enable 3-point calibration");
  Serial.println("[Cmd]       'cal off' -> skip calibration (normal reading)");
  Serial.println("[Cmd]       'cal print' -> show current coeff");
  Serial.println("[Cmd]       'cal reset' -> clear coeff & redo later");

  
  // 完成温度校准程序初始化
}

enum class CalState {
  IDLE, WAIT1, DO1, WAIT2, DO2, WAIT3, DO3, FIT, DONE
};

static CalState cal_state = CalState::WAIT1;
static double t1_true = 0.0;   // 缺省 0℃
static double t2_true = 25.0;  // 缺省 25℃
static double t3_true = 50.0;  // 缺省 50℃

static TempService::CalibCoeff g_coeff;
static TempService::CalibPoint s_p1, s_p2, s_p3;
static bool cal_once = false;

void loop() {
  if (!adc) {
    return;
  }
    // —— 电导命令优先处理：只在行首是 'c' 时拦截 —— 
  if (Serial.available() && (char)Serial.peek() == 'c') {
    String line = Serial.readStringUntil('\n');
    line.trim();
    extern void handleCondCommand(const String& line); // 前置声明
    handleCondCommand(line);
    return; // 处理完这轮 loop
  }

  if (!cal_enabled && Serial.available()) {
  String cmd = Serial.readStringUntil('\n');
  cmd.trim(); cmd.toLowerCase();

  if (cmd == "cal on") {
    cal_enabled = true;
    cal_once = false;
    cal_state = CalState::WAIT1;
    Serial.println("[Cal] ENABLED. Enter point #1 true T (e.g., 0 or 23.7), then 'y' to sample.");
    return;
  } else if (cmd == "cal off") {
    cal_enabled = false;
    Serial.println("[Cal] DISABLED. Running normal measurement.");
    return;
  } else if (cmd == "cal print") {
    if (g_tempSvc) {
      auto c = g_tempSvc->getCalib();
      if (c.valid) {
        Serial.printf("[Cal] Coeff: a=%.6g, b=%.6g, c=%.6g\n", c.a, c.b, c.c);
      } else {
        Serial.println("[Cal] No valid coeff yet.");
      }
    }
    return;
  } else if (cmd == "cal reset") {
    if (g_tempSvc) {
      TempService::CalibCoeff empty; // 默认 valid=false
      g_tempSvc->setCalib(empty);
    }
    cal_once = false;
    Serial.println("[Cal] Coeff cleared. Use 'cal on' to re-calibrate.");
    return;
  }
}



if (cal_enabled && !cal_once && g_tempSvc) {
    switch (cal_state) {
      case CalState::WAIT1:
        if (Serial.available()) {
          String s = Serial.readStringUntil('\n'); // 读到回车
          s.trim();
          if (s.equalsIgnoreCase("y")) {
            cal_state = CalState::DO1;
            Serial.printf("[Cal] Sampling point#1 at T_true=%.3fC ...\n", t1_true);
          } else if (s.length()) {
            t1_true = s.toFloat(); // 输入数字即更新“真实温度”
            Serial.printf("[Cal] Set point#1 T_true=%.3fC. Type 'y' to sample.\n", t1_true);
          }
           if (s.equalsIgnoreCase("x")) {
           cal_enabled = false;
           Serial.println("[Cal] CANCELED. Back to normal measurement.");
            return;
          }
        }
        return;
      case CalState::DO1: {
        TempService::CalibPoint p; 
        if (g_tempSvc->collectCalibPoint(t1_true, 64, p))  {
          s_p1 = p;
          Serial.printf("[Cal] p1: true=%.3f, meas=%.3f\n", p.t_true, p.t_meas);
          Serial.println("Put probe into point#2 (e.g., room 25.0C), then 'y'.");
          cal_state = CalState::WAIT2;
        } else {
          Serial.println("[Cal] p1 failed, retry: 'y'");
          cal_state = CalState::WAIT1;
        }
        return;
      }
      case CalState::WAIT2:
        if (Serial.available()) {
          String s = Serial.readStringUntil('\n');
          s.trim();
          if (s.equalsIgnoreCase("y")) {
            cal_state = CalState::DO2;
            Serial.printf("[Cal] Sampling point#2 at T_true=%.3fC ...\n", t2_true);
          } else if (s.length()) {
            t2_true = s.toFloat();
            Serial.printf("[Cal] Set point#2 T_true=%.3fC. Type 'y' to sample.\n", t2_true);
          }
           if (s.equalsIgnoreCase("x")) {
           cal_enabled = false;
           Serial.println("[Cal] CANCELED. Back to normal measurement.");
            return;
          }
        }
        return;
      case CalState::DO2: {
        
        TempService::CalibPoint p; 
        if (g_tempSvc->collectCalibPoint(t2_true, 64, p)) {
          s_p2 = p;
          Serial.printf("[Cal] p2: true=%.3f, meas=%.3f\n", p.t_true, p.t_meas);
          Serial.println("Put probe into point#3 (e.g., warm 50.0C), then 'y'.");
          cal_state = CalState::WAIT3;
        } else {
          Serial.println("[Cal] p2 failed, retry: 'y'");
          cal_state = CalState::WAIT2;
        }
        return;
      }
      case CalState::WAIT3:
        if (Serial.available()) {
          String s = Serial.readStringUntil('\n');
          s.trim();
          if (s.equalsIgnoreCase("y")) {
            cal_state = CalState::DO3;
            Serial.printf("[Cal] Sampling point#3 at T_true=%.3fC ...\n", t3_true);
          } else if (s.length()) {
            t3_true = s.toFloat();
            Serial.printf("[Cal] Set point#3 T_true=%.3fC. Type 'y' to sample.\n", t3_true);
          }
          if (s.equalsIgnoreCase("x")) {
           cal_enabled = false;
           Serial.println("[Cal] CANCELED. Back to normal measurement.");
          return;
          }
        }
        return;
        case CalState::DO3: {
          TempService::CalibPoint p; 
          if (g_tempSvc->collectCalibPoint(t3_true, 64, p)) {
            s_p3 = p;
            if (TempService::fitThreePoint(s_p1, s_p2, s_p3, g_coeff)) {
              g_tempSvc->setCalib(g_coeff);
              Serial.printf("[Cal] Done. a=%.6g, b=%.6g, c=%.6g\n", g_coeff.a, g_coeff.b, g_coeff.c);
              // 保存一次就够
              if (StorageManager::saveTempCalib(g_coeff))
                Serial.println("[Cal] Coeff saved to NVS.");
              else
                Serial.println("[Cal] Save to NVS FAILED!");

              cal_state = CalState::DONE; 
              cal_once = true;
            } else {
              Serial.println("[Cal] fit failed, restart from point#1: 'y'");
              cal_state = CalState::WAIT1;
            }
          } else {
            Serial.println("[Cal] p3 failed, retry: 'y'");
            cal_state = CalState::WAIT3;
          }
          return;
        }

      default: break;
    }
  }


  if (adc->waitDRDY(100)) {
    int32_t raw_value;
    
    if (adc->readData(raw_value)) {
      // defaultConfig 中设置的增益是 2
      int gain = 2; 
      // 使用 codeToVolt 将原始码值转换为被测电压 (V_RTD)
      double voltage = ADS124S08_Drv::codeToVolt(raw_value, V_REF, gain);

      // 根据公式反推电阻值
      // R_RTD = R_REF * (Output Code) / (Gain * 2^23)
      //       = (R_REF / Gain) * (Output Code / 2^23)
      //       = (V_REF / IDAC / Gain) * (Output Code / 2^23)
      // 注意: voltage = V_REF * (Output Code / (Gain * 2^23))
      // 所以: R_RTD = voltage * R_REF / V_REF = voltage / IDAC
      double resistance = voltage / 0.0005; // 500µA = 0.0005A

      Serial.print("Raw: ");
      Serial.print(raw_value);
      Serial.print("\t | V_RTD: ");
      Serial.print(voltage, 6);
      Serial.print(" V\t | R_RTD: ");
      Serial.print(resistance, 3);
      Serial.println(" Ohm");

      double t_meas = TempService::pt1000_R_to_T(resistance);
      double t_cal  = g_tempSvc ? g_tempSvc->applyCalib(t_meas) : t_meas;
      Serial.printf("T_meas: %.3f C | T_cal: %.3f C\n", t_meas, t_cal);
    } else {
      Serial.println("Failed to read data from ADS124S08.");
    }

  } else {
    Serial.println("Timeout: DRDY pin did not go low.");
  }

  delay(500);
}

static void printCondHelp_() {
  Serial.println("cond help");
  Serial.println("cond init [freqHz] [ampCode] [dftN]");
  Serial.println("cond rcal [R_ohm]");
  Serial.println("cond read [Kcell_1_per_cm]");
}

void handleCondCommand(const String& line) {
  if (!g_condSvc) { Serial.println("[COND] service not ready"); return; }

  // 统一处理：去空白 + 转小写做匹配
  String s = line; s.trim();
  String slow = s; slow.toLowerCase();

  if (!slow.startsWith("cond")) { printCondHelp_(); return; }

  // --- cond help ---
  if (slow == "cond" || slow.startsWith("cond help")) {
    printCondHelp_();
    return;
  }

  // --- cond init [freqHz] [ampCode] [dftN] ---
  if (slow.startsWith("cond init")) {
    double f = g_condCfg.freq_hz;
    unsigned amp = g_condCfg.amp_code;
    unsigned N = g_condCfg.dft_n;

    // 用 sscanf 抓可选参数（ESP32 的 libc 支持 float）
    // 允许 0~3 个参数：只填前面的；没填就用旧值
    int got = 0;
    // 去掉前缀，保留参数串
    String arg = s.substring(String("cond init").length());
    arg.trim();
    if (arg.length() > 0) {
      // 尝试 3 个
      got = sscanf(arg.c_str(), "%lf %u %u", &f, &amp, &N);
      if (got <= 0) {
        Serial.println("Usage: cond init [freqHz] [ampCode] [dftN]");
        return;
      }
      if (got == 1) { /*只更新f*/ }
      if (got == 2) { /*更新f,amp*/ }
      if (got >= 3) { /*更新f,amp,N*/ }
    }

    g_condCfg.freq_hz  = (float)f;
    g_condCfg.amp_code = (uint16_t)(amp & 0x7FF);
    g_condCfg.dft_n    = (uint32_t)N;
    g_condCfg.high_bw  = (g_condCfg.freq_hz >= 80000.0f);

    if (g_condSvc->reconfig(g_condCfg)) {
      Serial.printf("[COND] reconfig ok: f=%.1f Hz, amp=0x%03X (%u), N=%u, high_bw=%d\n",
                    g_condCfg.freq_hz, g_condCfg.amp_code, g_condCfg.amp_code,
                    g_condCfg.dft_n, g_condCfg.high_bw);
      Serial.println("[COND] NOTE: reconfig resets calibration; run:  cond rcal <R>");
    } else {
      Serial.println("[COND] reconfig failed");
    }
    return;
  }

  // --- cond rcal [R] ---
  if (slow.startsWith("cond rcal")) {
    double R = g_condCfg.Rcal_ohm;
    String arg = s.substring(String("cond rcal").length());
    arg.trim();
    if (arg.length() > 0) {
      int got = sscanf(arg.c_str(), "%lf", &R);
      if (got != 1) { Serial.println("Usage: cond rcal <R_ohm>"); return; }
    }
    g_condCfg.Rcal_ohm = R;
    Serial.printf("[COND] calibrating with R=%.3f ohm ...\n", g_condCfg.Rcal_ohm);
    if (g_condSvc->calibrate()) {
      Serial.printf("[COND] calib ok. C=%.6e (R*M)\n", g_condSvc->calibC());
    } else {
      Serial.println("[COND] calib failed (DFT not ready or read error)");
    }
    return;
  }

  // --- cond read [Kcell] ---
  if (slow.startsWith("cond read")) {
    double K = g_condCfg.cell_K;
    String arg = s.substring(String("cond read").length());
    arg.trim();
    if (arg.length() > 0) {
      int got = sscanf(arg.c_str(), "%lf", &K);
      if (got == 1) g_condCfg.cell_K = K;
    }
    CondReading r{};
    if (g_condSvc->read_once(r)) {
      Serial.printf("[COND] DFT: Re=%ld, Im=%ld, |M|=%.3f, phase=%.3f rad\n",
                    (long)r.real_raw, (long)r.imag_raw, r.mag, r.phase_rad);
      Serial.printf("[COND] |Z|=%.4f ohm,  G=%.6f S,  sigma=%.6f S/cm (K=%.3f 1/cm)\n",
                    r.Z_abs_ohm, r.G_si, r.sigma_Spcm, g_condCfg.cell_K);
    } else {
      Serial.println("[COND] read failed (timeout or read error)");
    }
    return;
  }

  // 未知子命令
  printCondHelp_();
}
