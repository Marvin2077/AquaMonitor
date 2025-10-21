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

/* ====== 共用 SPI 线 ====== */
static const int PIN_SCLK = 14;
static const int PIN_MISO = 12;
static const int PIN_MOSI = 13;

/* ====== 片选/辅助 ====== */
// ADS124S08
static const int CS_ADS    = 15;
static const int DRDY_ADS  = 20; // 数据手册中为 GPIO4，请根据您的板子确认
static const int RESET_ADS = 27;
// ====== AD5941 片选/辅助 ======
static const int CS_AD5941 = 26; // 假设 AD5941 的 CS 连接到 GPIO 26
static const int RESET_AD5941 = 25; // 假设 RESET 连接到 GPIO 25 (如果连接了)
// static const int INT_AD5941 = 34;   // 假设中断输出连接到 GPIO 34 (仅输入引脚)

// 定义外部参考电压值，用于后续电压转换计算 请根据您实际连接到 REFP0/REFN0 的参考电压修改此值
const double V_REF = 1.57; 

// 创建一个 ADS124S08 驱动对象（此时还未初始化）
ADS124S08_Drv* adc = nullptr;
// 创建 AD5941 驱动对象指针
AD5941_Drv* afe = nullptr;
//创建 Temp Service 对象指针
TempService* g_tempSvc = nullptr;

// === Calibration control ===
static bool cal_enabled = false;   // 默认不进校准流程
// 可选：启动就想进校准可改为 true

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

    // 初始化温度校准程序
  TempService::Config tcfg;
  tcfg.Rref_ohm = V_REF / 0.0005;  // 1.57V / 500uA ≈ 3140 Ω
  tcfg.pga_gain = 2;      // 与 defaultConfig 的增益保持一致
  static TempService tempSvc(*adc, tcfg);
  g_tempSvc = &tempSvc;
  Serial.println("[Cal] Ready for 3-point calibration.");
  Serial.println("Put probe into point #1 (e.g., 0.0C), then type 'y' + Enter.");
  Serial.println("Tip: You can type a number (e.g., 23.7) to set the true temperature, then type 'y' to sample.");
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
            cal_state = CalState::DONE; cal_once = true;
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