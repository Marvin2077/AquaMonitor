#include <Arduino.h>
#include <SPI.h>

#include "storage_manager.h"
#include "mux_iface.h"
#include "spi_hal.h"        
//芯片驱动与Glue头文件
#include "ads124s08_board_glue.h"    
#include "ads124s08_drv.h"  
extern "C" {
#include "ad5940.h"
}
#include "ad5941_board_glue.h"   
#include "ad5941PlatformCfg.h"
//服务头文件
#include "temp_service.h" 
#include "Conductivity_service.h"
#include "ph_service.h"

/* ====== 共用 SPI 总线引脚 ====== */
static const int PIN_SCLK = 14; // ESP32 SPI 时钟线
static const int PIN_MISO = 12; // ESP32 SPI MISO线
static const int PIN_MOSI = 13; // ESP32 SPI MOSI线
/* ====== AD5941 特定引脚 ====== */
static const int CS_AD5941    = 27; // AD5941 片选引脚
static const int RESET_AD5941 = 26; // AD5941 复位引脚
/* ====== ADS124S08 特定引脚 ====== */
static const int CS_ADS124S08    = 16; // ADS124S08 片选引脚
static const int RESET_ADS124S08 = 15; // ADS124S08 复位引脚
static const int DRDY_ADS124S08 =  4; // ADS124S08 DRDY引脚
/* ====== Deafult设备ID ====== */
int Deviceid = -1;
//解析串口命令函数
void handleSerialCommand();
// === AD5941 全局变量 ===
// 定义用于存放 AD5941 序列器指令的缓冲区
#define APPBUFF_SIZE 512
uint32_t AppBuff[APPBUFF_SIZE];
// === 电导率服务标志位/全局变量 ===
// false = 日常单频模式 (1000Hz)
// true  = 正在进行扫频
bool g_isCondMode = false;
bool g_isSweepMode = false;
int g_sweepCount = 0;     // 当前已测量的点数
int g_sweepTotalPoints = 0; // 总共要测的点数
float g_condStdValue = 1413.0f; // 默认标准液值
bool g_isCalibratingCond = false;
// ===pH 服务标志位/全局变量 ===
bool g_ispHMode = false;
bool g_isCalibratingOffset = false; // 是否正在进行 Offset 校准
bool g_isCalibratingGain = false;   // 是否正在校准增益
float g_calResistorValue = 1000.0f; // 用户输入的校准电阻阻值 (默认为 1k)
// === ADS124S08 全局变量 ===
// 创建一个 ADS124S08 驱动对象（此时还未初始化）
ADS124S08_Drv* ads124s08 = nullptr;
// ===Temperature 服务标志位/全局变量 ===
TempService* g_tempSvc = nullptr;
const double MY_RREF = 3300.0;
bool is_calibrating = false;
double currentTemp = 0.0;
bool g_isTempMode = false;
// === 有限状态机变量 ===
enum SystemState {
  STATE_IDLE,             // 空闲状态
  STATE_TEMP_MEASURE,     // 温度测量
  STATE_TEMP_CAL_P1,      // 温度校准点1
  STATE_TEMP_CAL_P2,      // 温度校准点2
  STATE_TEMP_CAL_P3,      // 温度校准点3
  STATE_TEMP_SAVE_CAL,    // 温度保存校准
  STATE_TEMP_RESET_CAL,   // 温度重置校准
  STATE_COND_INIT,        // 电导率初始化
  STATE_COND_MEASURE,     // 电导率测量
  STATE_COND_SWEEP,       // 电导率扫频
  STATE_COND_CAL,         // 电导率电极常数校准
  STATE_PH_INIT,        // 电导率初始化
  STATE_PH_MEASURE,       // pH 测量
  STATE_PH_CAL_OFFSET,    // pH Offset 校准
  STATE_PH_CAL_GAIN       // pH Gain 校准
};
// 当前状态变量，初始化为空闲
SystemState currentState = STATE_IDLE;

void setup() {
  //读取Device id
  Serial.begin(115200);
  ensureDeviceID();
  delay(1000);
  //初始化MUX
  MUXPinInit();
  ChooseSenesingChannel(1);
  ChooseISFETChannel(1);

  //初始化 SPI 总线
  SpiHAL::beginBus(PIN_SCLK, PIN_MISO, PIN_MOSI);
  Serial.println("SPI bus initialized.");
  // --- 初始化 AD5941 --- //
  Serial.println("======================================");
  Serial.println(" AD5941 Setup Start ");
  Serial.println("======================================");
  
  // 1) 创建 AD5941 的 SpiDevice 对象
  static SpiDevice ad5941_spidev(CS_AD5941, 8000000 /* 8MHz */, MSBFIRST, SPI_MODE0);
  Serial.println("SpiDevice for AD5941 created.");

  // 2) 配置并初始化 AD5941 Glue 层
  Ad5941Glue::Config cfg;
  cfg.spi = &ad5941_spidev;
  cfg.pin_reset = RESET_AD5941;
  Ad5941Glue::setup(cfg);
  Serial.println("Ad5941Glue setup complete.");
  // 3) 执行硬件复位
  Ad5941Glue::hardware_reset(1000, 100);
  Serial.println("AD5941 hardware reset performed.");
  if(AD5941PlatformCfg() == AD5940ERR_OK)
  {
    Serial.println("AD5941 Platform Configuration performed.");
  }

  // --- 初始化 ADS124S08 --- //
  Serial.println("======================================");
  Serial.println(" ADS124S08 Setup Start ");
  Serial.println("======================================");
   // 1)  初始化 ADS124S08 SPI 总线
  static SpiDevice ads_spidev(CS_ADS124S08, 4000000, MSBFIRST, SPI_MODE1);
  Serial.println("SpiDevice for AD5941 created.");
  //  2)  定义 ADS124S08 的 Glue 配置
  AdsGlueConfig ads_cfg = {
    .spi = &ads_spidev,
    .pin_drdy = DRDY_ADS124S08,
    .pin_reset = RESET_ADS124S08
  };
  Serial.println("Making ADS124S08 HAL...");
  ADS124S08_Drv::Hal ads_hal = BoardGlue::make_ads_hal(ads_cfg);
  // 3) 创建ADS124S08驱动实例
  ads124s08 = new ADS124S08_Drv(ads_hal);
  Serial.println("ADS124S08 Driver instance created.");
  Serial.println("Configuring ADS124S08...");
  if (!ads124s08->defaultConfig()) {
    Serial.println("!!! ADS124S08 configuration FAILED! Halting. !!!");
    while (1) { delay(1000); }
  }
  // --- 完成ADS124S08芯片的初始化 --- //

  // --- 初始化 ADS124S08 温度测量服务 --- //
  TempService::Config tsCfg;
  tsCfg.Rref_ohm = MY_RREF; 
  tsCfg.pga_gain = 2;      
  g_tempSvc = new TempService(*ads124s08, tsCfg);
  Serial.println("ADS124S08 configuration successful.");
  // --- 初始化 AD5941 服务结构体 --- //

  //初始化应用参数结构体
  AppCondCfg_init();
  AppPHCfg_init();
  // 1. 读取并应用电导率参数
  float saved_K = loadCondParams();
  AppCondCfg.K_Cell = saved_K;
  Serial.printf("[Setup] Loaded Cond K_Cell: %.4f\n", saved_K);

  // 2. 读取并应用 pH 参数
  PhCalibData phData = loadPhParams();
  AppPHCfg.ZeroOffset_Code = phData.offsetCode;
  AppPHCfg.Rtia_Value_Ohm = phData.rtiaVal;
  Serial.printf("[Setup] Loaded pH Offset: %d, Rtia: %.2f\n", phData.offsetCode, phData.rtiaVal);

  // 3. 读取并应用 温度 参数
  // 假设你的 g_tempSvc 有一个方法叫 setCalib
  TempCalibData tempData = loadTempParams();
  if (tempData.valid) {
     TempService::CalibCoeff c;
     c.a = tempData.a;
     c.b = tempData.b;
     c.c = tempData.c;
     // c.valid = true; // 如果你的结构体里有 valid
     g_tempSvc->setCalib(c); 
     Serial.println("[Setup] Loaded Temp Calibration.");
  } else {
     Serial.println("[Setup] No valid Temp Calibration found.");
  }
  //完成芯片和服务初始化
  Serial.println("System Initialized");
}


void loop() {
  static uint32_t last_time = 0;
  uint32_t tempCount = APPBUFF_SIZE;
  
  handleSerialCommand();
  // 定时触发部分：每 500ms
  if (millis() - last_time > 500) {
    last_time = millis();
  }
  switch(currentState)
  {
    case STATE_IDLE:
      break;
    case STATE_TEMP_MEASURE:
      g_tempSvc->measure(currentTemp);
      Serial.printf(" Water Temp: %3f \n", currentTemp);
      currentState = STATE_IDLE;
      break;
    case STATE_TEMP_CAL_P1:
      if (g_tempSvc->recordCalibPoint(0, 25.0)) 
      {
         Serial.println("Point 1 Saved!");
      } else 
      {
         Serial.println("Point 1 Failed!");
      }
      currentState = STATE_IDLE;
      break;
    case STATE_TEMP_CAL_P2:
      if (g_tempSvc->recordCalibPoint(0, 50.0)) 
      {
         Serial.println("Point 2 Saved!");
      } else 
      {
         Serial.println("Point 2 Failed!");
      }
      currentState = STATE_IDLE;
      break;
    case STATE_TEMP_CAL_P3:
      if (g_tempSvc->recordCalibPoint(0, 80.0)) 
      {
         Serial.println("Point 3 Saved!");
      } else 
      {
         Serial.println("Point 3 Failed!");
      }
      currentState = STATE_IDLE;
      break;
    case STATE_TEMP_SAVE_CAL:
      if (g_tempSvc->finishCalibration()) {
        auto c = g_tempSvc->getCalib();
        saveTempParams(c.a, c.b, c.c, true);
        Serial.printf("Calibration DONE! a=%.6f, b=%.6f, c=%.6f\n", c.a, c.b, c.c);
      } else {
        Serial.println("Calibration Calculation Failed (Check points?)");
      }
      currentState = STATE_IDLE;
    break;
    case STATE_TEMP_RESET_CAL:
      {
        TempService::CalibCoeff clean; // 默认是 valid=false
        g_tempSvc->setCalib(clean);
        Serial.println("Calibration Cleared.");
        currentState = STATE_IDLE;
      }
    break;
    // === 处理电导率服务 ===
    case STATE_COND_INIT:
      g_isCondMode = true;
      g_ispHMode = false;
      ChooseSenesingChannel(1);
      if(AppCondInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK) 
      {
        AppPHCfg.PHInited = bFALSE;
        Serial.println("pH Service Init OK!");
      }
      else 
      {
        Serial.println("Conductivity Service Init FAILED!");
        currentState = STATE_IDLE;
        break;
      } 
      currentState = STATE_IDLE;
    break;
    case STATE_COND_MEASURE:
      if(AppCondCfg.CondInited == bFALSE || g_isCondMode == false)
      {
        Serial.println("Conductivity Service haven't been initialized");
        currentState = STATE_IDLE;
        break;
      }
      if (AppCondISR(AppBuff, &tempCount) == 0) 
      {
        if (tempCount > 0) 
        {
          CondShowResult(AppBuff, tempCount);
          currentState = STATE_IDLE;        
        }
      }
    break;
    case STATE_COND_SWEEP:
      if(AppCondCfg.CondInited == bFALSE || g_isCondMode == false)
      {
        Serial.println("Conductivity Service haven't been initialized");
        currentState = STATE_IDLE;
        break;
      }
      if(AppCondISR(AppBuff, &tempCount) == 0) 
      {
        if(tempCount > 0) 
        {
          CondShowResult(AppBuff, tempCount);
          g_sweepCount++;
          if (g_sweepCount < g_sweepTotalPoints) 
            {
              AppCondCtrl(CondCTRL_START, 0); 
              Serial.printf("\n");
            } 
          else 
            {
              Serial.println(">>> Sweep Completed! <<<");
              g_isSweepMode = false;
              AppCondCfg.SinFreq = AppCondCfg.SinFreq; // 设回你的默认频率
              AppCondCfg.SweepCfg.SweepEn = bFALSE;
              AppCondCfg.bParaChanged = bTRUE;
              AppCondInit(AppBuff, APPBUFF_SIZE); 
              currentState = STATE_IDLE;
            }
        }
      }
    break;
    case STATE_COND_CAL:
      if(AppCondCfg.CondInited == bFALSE || g_isCondMode == false)
      {
        Serial.println("Conductivity Service haven't been initialized");
        currentState = STATE_IDLE;
        break;
      }

      if(AppCondISR(AppBuff, &tempCount) == 0)
      {
        if(tempCount > 0) 
        {
          float measured_G = ComputeKCell(AppBuff, tempCount);
          Serial.printf("Conductivity: %f \n",measured_G);
          float new_K = g_condStdValue / measured_G;
          AppCondCfg.K_Cell = new_K;
          saveCondParams(new_K);
          Serial.println(">>> Calibration Successful! <<<");
          Serial.printf("Std Solution: %.2f uS/cm\n", g_condStdValue);
          Serial.printf("Measured G:   %.ascribed4f uS\n", measured_G);
          Serial.printf("New K_Cell:   %.4f cm^-1\n", new_K);
          currentState = STATE_IDLE;
          }
      }
    break;
    // === 处理pH服务 ===
    case STATE_PH_INIT:
      ChooseSenesingChannel(2);
      g_isCondMode = false;
      g_ispHMode = true;
      if(AppPHInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK) 
      {
        AppCondCfg.CondInited = bFALSE;
        Serial.println("pH Service Init OK!");
      }
      else 
      {
        Serial.println("pH Service Init FAILED!");
        currentState = STATE_IDLE;
        break;
      } 
      currentState = STATE_IDLE;
    break;
    case STATE_PH_MEASURE:
      if(AppPHCfg.PHInited == bFALSE || g_ispHMode == false )
      {
        Serial.println("pH Service haven't been initialized");
        currentState = STATE_IDLE;
        break;
      }
      if (AppPHISR(AppBuff, &tempCount) == 0) 
      {
        if (tempCount > 0) 
        {
          Serial.println("pH Service Init FAILED!");
          PHShowResult(AppBuff, tempCount);
          currentState = STATE_IDLE;
        }
      }
    break;
    case STATE_PH_CAL_OFFSET:
      if(AppPHCfg.PHInited == bFALSE || g_ispHMode == false )
      {
        Serial.println("pH Service haven't been initialized");
        currentState = STATE_IDLE;
        break;
      }
      if (AppPHISR(AppBuff, &tempCount) == 0) 
      {
        if (tempCount > 0) 
        {
          // 取第一个点作为 Offset 
          uint16_t measured_offset = AppBuff[0] & 0xFFFF;            
          // 保存到配置中
          AppPHCfg.ZeroOffset_Code = measured_offset;
          savePhParams(measured_offset, AppPHCfg.Rtia_Value_Ohm);
          Serial.printf(">>> Offset Calibrated! New Zero Code: 0x%04X (%d) <<<\n", measured_offset, measured_offset);
          // 校准完成后，自动恢复开关设置到正常模式
          AppPHCfg.TswitchSel = SWT_AIN0 | SWT_TRTIA; // 恢复连接传感器
          AppPHCfg.bParaChanged = bTRUE;
          AppPHInit(AppBuff, APPBUFF_SIZE); // 重建序列
          currentState = STATE_IDLE;
        }
      }
    break;
    case STATE_PH_CAL_GAIN:
      if(AppPHCfg.PHInited == bFALSE || g_ispHMode == false )
      {
        Serial.println("pH Service haven't been initialized");
        currentState = STATE_IDLE;
        break;
      }
      if (AppPHISR(AppBuff, &tempCount) == 0) 
      {
        if (tempCount > 0) 
        {
          // 1. 获取原始 Code (取第0个点)
          uint16_t rawCode = AppBuff[0] & 0xFFFF;
          // 2. 计算去 Offset 后的电压绝对值
          // 使用你之前校准好的 ZeroOffset_Code
          int32_t diff_code = (int32_t)rawCode - (int32_t)AppPHCfg.ZeroOffset_Code;
          // 1.82V 是量程，32768 是半满量程
          float voltage_diff = ((float)diff_code / 32768.0f) * 1.82f;
          float abs_volt = fabs(voltage_diff);
          // 3. 反推真实 RTIA
          // 原理: |Volt| = I_ideal * R_tia_real
          //       I_ideal = 1.1V / R_ext
          // 所以: R_tia_real = (|Volt| * R_ext) / 1.1V
          if (abs_volt > 0.05f) // 确保有足够电压，防止除零或噪声干扰
          { 
            float calculated_rtia = (abs_volt * g_calResistorValue) / 1.1f;
            // 4. 更新系统参数
            AppPHCfg.Rtia_Value_Ohm = calculated_rtia;
            savePhParams(AppPHCfg.ZeroOffset_Code, calculated_rtia);
            Serial.printf(">>> Gain Calibrated! Raw:0x%04X, Vdiff:%.4fV, New RTIA: %.2f Ohm <<<\n", 
            rawCode, voltage_diff, calculated_rtia);
          }
          else 
          {
            Serial.printf(">>> Error: Signal too low (%.4fV). Is resistor connected? <<<\n", abs_volt);
          }
        }

      }
    break;
  }
}


void handleSerialCommand() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); // 去掉回车换行

    // === 温度指令 ===
    // === 指令: "temp read" -> 读取一次温度值 ===
    if(cmd == "temp read")
    {
      Serial.println("[CMD] Measuring Temperature");
      currentState = STATE_TEMP_MEASURE;
    }
    // === 指令: "temp cal 0" -> 校准 0度点 ===
    else if (cmd == "temp cal 25") {
      Serial.println("[CMD] Measuring Point 1 (25.0 C)... Keep sensor stable.");
      currentState = STATE_TEMP_CAL_P1;
    }
    
    // === 指令: "temp cal 50" -> 校准 50度点 ===
    else if (cmd == "temp cal 50") {
      Serial.println("[CMD] Measuring Point 2 (50.0 C)...");
      currentState = STATE_TEMP_CAL_P2;
    }

    // === 指令: "temp cal 100" -> 校准 100度点 ===
    else if (cmd == "temp cal 80") {
      Serial.println("[CMD] Measuring Point 3 (80.0 C)...");
      currentState = STATE_TEMP_CAL_P3;
    }

    // === 指令: "temp save" -> 计算并生效 ===
    else if (cmd == "temp save") {
      Serial.println("[CMD] Computing temperature calibration coefficients...");
      currentState = STATE_TEMP_SAVE_CAL;
    }

    // === 指令: "temp reset" -> 清除温度校准 ===
    else if (cmd == "temp reset") {
      Serial.println("[CMD] Resetting temperature calibration coefficients...");
      currentState = STATE_TEMP_RESET_CAL;
    }

    // === 电导率指令 ===
    // === 指令: "cond read" -> 触发一次阻抗测量 ===
    else if (cmd == "cond init") {
      Serial.println("[CMD] Init conductivity service...");      
      currentState = STATE_COND_INIT;
    }    

    else if (cmd == "cond read") {
      if (!g_isCondMode) 
      {
        Serial.println("Not in Conductivity Mode");
        return;
      }
      ChooseSenesingChannel(1);
      Serial.println("[CMD] Triggering Cond_Impedance Measurement...");
      AppCondCtrl(CondCTRL_START, 0);
      currentState = STATE_COND_MEASURE;
    }    

    else if (cmd == "cond cal") {
      if (!g_isCondMode) 
      {
        Serial.println("Not in Conductivity Mode");
        return;
      }
      Serial.println("[CMD] Starting K_Cell calibration");
      ChooseSenesingChannel(1);
      AppCondCtrl(CondCTRL_START, 0);
      currentState = STATE_COND_CAL;
    } 

    else if (cmd == "cond sweep") {
      if (g_isSweepMode) 
      {
        Serial.println("Already sweeping!");
        return;
      }
      if (!g_isCondMode) 
      {
        Serial.println("Not in Conductivity Mode");
        return;
      }
      ChooseSenesingChannel(1);
      Serial.println("[CMD] Starting Frequency Sweep (1kHz -> 100kHz)...");
      AppCondCfg.SweepCfg.SweepEn = bTRUE;
      AppCondCfg.ReDoRtiaCal = bTRUE;
      AppCondCfg.bParaChanged = bTRUE;   
      if (AppCondInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK) 
      {
          g_isSweepMode = true;
          g_sweepCount = 0; 
          g_sweepTotalPoints = AppCondCfg.SweepCfg.SweepPoints;
          currentState = STATE_COND_SWEEP;
          AppCondCtrl(CondCTRL_START, 0);
       } else {
           Serial.println("Sweep Init Failed!");
       }
    }
    // === pH值指令 ===
    else if (cmd == "ph init") 
    {
      ChooseSenesingChannel(2);
      Serial.println("[CMD] Initializing pH Measurement...");
      currentState = STATE_PH_INIT;
    }
    else if (cmd == "ph read") 
    {
      if (!g_ispHMode) 
      {
        Serial.println("Not in pH Mode");
        return;
      }
      ChooseSenesingChannel(2);
      Serial.println("[CMD] Triggering Single pH Measurement...");
      currentState = STATE_PH_MEASURE;
      AppPHCtrl(PHCTRL_START, 0);
    }
    else if (cmd == "ph cal offset") {
      if (!g_ispHMode) 
      {
        Serial.println("Not in pH Mode");
        return;
      }
      ChooseSenesingChannel(2);
      Serial.println("[CMD] Calibrating pH Offset (Disconnecting Input)...");
      // 1. 修改配置：断开所有开关
      AppPHCfg.TswitchSel =  SWT_TRTIA; 
      // 2. 标记参数已改变，并重新初始化序列
      AppPHCfg.bParaChanged = bTRUE;
      if (AppPHInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK) {
          // 3. 设置校准标志位，并触发一次测量
          g_isCalibratingOffset = true;
          AppPHCtrl(PHCTRL_START, 0);
          currentState = STATE_PH_CAL_OFFSET;
      } else {
          Serial.println("pH Cal Init Failed!");
      }
    }
    else if (cmd.startsWith("ph cal gain")) {
      if (!g_ispHMode) 
      {
        Serial.println("Not in pH Mode");
        return;
      }
      ChooseSenesingChannel(2);
        // 1. 解析输入的电阻值
        String valStr = cmd.substring(11); // "ph cal gain" 长度是 11
        valStr.trim();
        if (valStr.length() > 0) {
            float r_val = valStr.toFloat();
            if (r_val > 0) {
                g_calResistorValue = r_val;
                Serial.printf("[CMD] Calibrating Gain with R_ext = %.1f Ohm...\n", g_calResistorValue);

                // 2. 确保开关是连接状态 (AIN0 + TRTIA)
                // 如果之前跑了 offset 校准，开关可能是断开的，这里要连回去
                AppPHCfg.DswitchSel = SWD_OPEN;
                AppPHCfg.PswitchSel = SWP_PL | SWP_PL2;
                AppPHCfg.NswitchSel = SWN_NL | SWN_NL2;
                AppPHCfg.TswitchSel = SWT_AIN0 | SWT_TRTIA; 
                // 3. 重新初始化序列以应用开关设置
                AppPHCfg.bParaChanged = bTRUE;
                if (AppPHInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK) {
                    // 4. 设置标志位并触发
                    AppPHCtrl(PHCTRL_START, 0);
                    currentState = STATE_PH_CAL_GAIN;
                } else {
                    Serial.println("Gain Cal Init Failed!");
                }
            } else {
                Serial.println("Invalid Resistor Value!");
            }
        } else {
            Serial.println("Usage: ph cal gain <ohms> (e.g., ph cal gain 1000)");
        }
    }
    // === 指令: "factory reset" -> 恢复默认校准参数 ===
    else if (cmd == "factory reset") {
      Serial.println("[CMD] Resetting to Factory Defaults...");
      // 1. 清除 Flash 中的数据
      resetCalibrationParams();
      // 2. 立即恢复 RAM 中的变量为默认值 (热重载)
      // --- 电导率默认值 ---
      AppCondCfg.K_Cell = 1.0f; 
      Serial.println("   Cond K_Cell -> 1.0");

      // --- pH 默认值 ---
      AppPHCfg.ZeroOffset_Code = 32768; // ADC半量程
      AppPHCfg.Rtia_Value_Ohm = AppCondCfg.HstiaRtiaSel;
      Serial.println("   pH Offset -> 32768, Gain -> 1000.0");

      // --- 温度默认值 ---
      TempService::CalibCoeff clean; // 默认构造函数通常是全0且valid=false
      g_tempSvc->setCalib(clean);
      Serial.println("   Temp Cal -> Cleared");
      // 3. 强制重新初始化各个服务，让参数生效
      //    (由于参数改了，需要告诉 AD5941 重新加载)
      Serial.println(">>> Factory Reset Complete. Please Restart or Init Sensors. <<<");
    }

  }
}