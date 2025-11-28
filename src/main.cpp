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
bool g_isSweepMode = false;
int g_sweepCount = 0;     // 当前已测量的点数
int g_sweepTotalPoints = 0; // 总共要测的点数
// ===pH 服务标志位/全局变量 ===
bool g_isPhMode = false;
bool g_isCalibratingOffset = false; // 是否正在进行 Offset 校准
bool g_isCalibratingGain = false;   // 是否正在校准增益
float g_calResistorValue = 1000.0f; // 用户输入的校准电阻阻值 (默认为 1k)
// === ADS124S08 全局变量 ===
// 创建一个 ADS124S08 驱动对象（此时还未初始化）
ADS124S08_Drv* ads124s08 = nullptr;
//创建 Temp Service 对象指针
TempService* g_tempSvc = nullptr;
const double MY_RREF = 3300.0;
bool is_calibrating = false;


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
  // --- 初始化 AD5941 阻抗测量服务 --- //
  //Serial.println("Initializing Impedance Service...");
  //初始化应用参数结构体
  //AppCondCfg_init();
  AppPHCfg_init();
  // 5. 配置2线应用参数 (修改Rcal和AIN1)
  if(AppPHInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK) {
    Serial.println("pH Service Init OK!");
  }
  else 
    {
        Serial.println("pH Service Init FAILED!");
    }
  // Serial.println("Running RTIA Calibration... This may take a moment.");
  // if(AppCondInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK) {
  //   Serial.println("Cond_Impedance Service Init OK!");
  // }
  // else 
  //   {
  //       Serial.println("Cond_Impedance Service Init FAILED!");
  //   }
  //完成芯片和服务初始化
  Serial.println("System Initialized");
}


void loop() {
  static uint32_t last_time = 0;
  uint32_t tempCount = APPBUFF_SIZE;
  
  handleSerialCommand();
  // 1. 触发部分：每 500ms
  if (millis() - last_time > 500) {
    last_time = millis();
     AppPHCtrl(PHCTRL_START,0);
    // 如果不是扫频模式，触发单点测量
    if (!g_isSweepMode) {
       //AppCondCtrl(CondCTRL_START, 0);
    }
  }
  if (g_isPhMode || g_isCalibratingOffset || g_isCalibratingGain) {
        if (AppPHISR(AppBuff, &tempCount) == 0) {
            if (tempCount > 0) {
                
                // === 分支 1：如果是校准模式 ===
                if (g_isCalibratingOffset) {
                    // 取第一个点作为 Offset (或者你可以做平均)
                    uint16_t measured_offset = AppBuff[0] & 0xFFFF;
                    
                    // 保存到配置中
                    AppPHCfg.ZeroOffset_Code = measured_offset;
                    
                    Serial.printf(">>> Offset Calibrated! New Zero Code: 0x%04X (%d) <<<\n", measured_offset, measured_offset);
                    
                    // 校准完成后，自动恢复开关设置到正常模式
                    AppPHCfg.TswitchSel = SWT_AIN0 | SWT_TRTIA; // 恢复连接传感器
                    AppPHCfg.bParaChanged = bTRUE;
                    AppPHInit(AppBuff, APPBUFF_SIZE); // 重建序列
                    
                    g_isCalibratingOffset = false; // 退出校准模式
                } 
                else if (g_isCalibratingGain) {
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
                  
                  if (abs_volt > 0.05f) { // 确保有足够电压，防止除零或噪声干扰
                      float calculated_rtia = (abs_volt * g_calResistorValue) / 1.1f;
                      
                      // 4. 更新系统参数
                      AppPHCfg.Rtia_Value_Ohm = calculated_rtia;
                      
                      Serial.printf(">>> Gain Calibrated! Raw:0x%04X, Vdiff:%.4fV, New RTIA: %.2f Ohm <<<\n", 
                                    rawCode, voltage_diff, calculated_rtia);
                  } else {
                      Serial.printf(">>> Error: Signal too low (%.4fV). Is resistor connected? <<<\n", abs_volt);
                  }
                  
                  // 退出校准模式
                  g_isCalibratingGain = false;
              }
                // === 分支 2：如果是普通测量模式 ===
                else {
                    PHShowResult(AppBuff, tempCount);
                    g_isPhMode = false;
                }
            }
        }
    }
  if (!g_isSweepMode) {
      // 检查 AD5941 是否有新数据
      if (AppCondISR(AppBuff, &tempCount) == 0) 
      {
        // 只有当真正读到数据时 (tempCount > 0)
        if (tempCount > 0) 
        {
          CondShowResult(AppBuff, tempCount);
          double currentTemp = 0.0;
          // 1. 此时再去测温度，保证时间和阻抗数据对齐
          if (g_tempSvc->measure(currentTemp)) {
             Serial.printf(" Water Temp: %3f \n", currentTemp);
          } else {
             Serial.println("Temp Read Failed!");
          }          
        }
      }
  } 
  else {
      // 如果是扫频模式，逻辑稍有不同，这里保留你原本的扫频处理
      if(AppCondISR(AppBuff, &tempCount) == 0) {
          if(tempCount > 0) {
              CondShowResult(AppBuff, tempCount);
              g_sweepCount++;
              if (g_sweepCount < g_sweepTotalPoints) {
                  AppCondCtrl(CondCTRL_START, 0); 
              } else {
                  Serial.println(">>> Sweep Completed! <<<");
                  g_isSweepMode = false;
                  // 恢复单点参数...
                  AppCondCfg.SinFreq = 2000.0; // 设回你的默认频率
                  AppCondCfg.SweepCfg.SweepEn = bFALSE;
                  AppCondCfg.bParaChanged = bTRUE;
                  AppCondInit(AppBuff, APPBUFF_SIZE); 
              }
          }
      }
  }
}


void handleSerialCommand() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); // 去掉回车换行

    // === 指令: "cal 0" -> 校准 0度点 ===
    if (cmd == "cal 0") {
      Serial.println("Measuring Point 1 (0.0 C)... Keep sensor stable.");
      // 假设这杯水是 0.0 度
      if (g_tempSvc->recordCalibPoint(0, 0.0)) {
         Serial.println("Point 1 Saved!");
      } else {
         Serial.println("Point 1 Failed!");
      }
    }
    
    // === 指令: "cal 50" -> 校准 50度点 ===
    else if (cmd == "cal 50") {
      Serial.println("Measuring Point 2 (50.0 C)...");
      if (g_tempSvc->recordCalibPoint(1, 50.0)) {
         Serial.println("Point 2 Saved!");
      }
    }

    // === 指令: "cal 100" -> 校准 100度点 ===
    else if (cmd == "cal 100") {
      Serial.println("Measuring Point 3 (100.0 C)...");
      if (g_tempSvc->recordCalibPoint(2, 100.0)) {
         Serial.println("Point 3 Saved!");
      }
    }

    // === 指令: "save" -> 计算并生效 ===
    else if (cmd == "save") {
      Serial.println("Computing coefficients...");
      if (g_tempSvc->finishCalibration()) {
        auto c = g_tempSvc->getCalib();
        Serial.printf("Calibration DONE! a=%.6f, b=%.6f, c=%.6f\n", c.a, c.b, c.c);
      } else {
        Serial.println("Calibration Calculation Failed (Check points?)");
      }
    }
    
    // === 指令: "reset" -> 清除校准 ===
    else if (cmd == "reset") {
      TempService::CalibCoeff clean; // 默认是 valid=false
      g_tempSvc->setCalib(clean);
      Serial.println("Calibration Cleared.");
    }
    // === 新增: "imp" -> 触发一次阻抗测量 ===
    else if (cmd == "Cond") {
       Serial.println("Triggering Cond_Impedance Measurement (1000Hz)...");
       // 所以这里调用它会直接让 AD5941 跑一次 SRAM 里的 SEQID_0 序列
       AppCondCtrl(CondCTRL_START, 0);
    }    

        else if (cmd == "Condsweep") {
       if (g_isSweepMode) {
           Serial.println("Already sweeping!");
           return;
       }
       
       Serial.println("[CMD] Starting Frequency Sweep (1kHz -> 100kHz)...");
       // 1. 配置参数
       AppCondCfg.SweepCfg.SweepEn = bTRUE;
       AppCondCfg.ReDoRtiaCal = bTRUE;
       AppCondCfg.bParaChanged = bTRUE;   
       // 3. 初始化服务 (这会计算出 SweepNextFreq = 2000.0)
       if (AppCondInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK) {
           g_isSweepMode = true;
           g_sweepCount = 0; 
           g_sweepTotalPoints = AppCondCfg.SweepCfg.SweepPoints;
           // 4. 触发第 1 个点 (1000Hz)
           AppCondCtrl(CondCTRL_START, 0);
       } else {
           Serial.println("Sweep Init Failed!");
       }
    }
    else if (cmd == "ph read") {
      Serial.println("Triggering Single pH Measurement...");
      // 1. 切换模式标志
      g_isPhMode = true;
      g_isSweepMode = false; // 确保不处于扫频模式
      AppPHCtrl(PHCTRL_START, 0);
  }
    else if (cmd == "ph cal offset") {
      Serial.println("[CMD] Calibrating pH Offset (Disconnecting Input)...");
      
      // 1. 修改配置：断开所有开关 (SWT_OPEN)
      // 这样 ADC 测量的就是系统本身的底噪/偏置

      AppPHCfg.TswitchSel =  SWT_TRTIA; 

      // 2. 标记参数已改变，并重新初始化序列
      AppPHCfg.bParaChanged = bTRUE;
      if (AppPHInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK) {
          // 3. 设置校准标志位，并触发一次测量
          g_isCalibratingOffset = true;
          g_isPhMode = false; // 暂停普通模式的打印
          AppPHCtrl(PHCTRL_START, 0);
      } else {
          Serial.println("Cal Init Failed!");
      }
    }
    
    // === 恢复正常测量模式指令 (可选) ===
    else if (cmd == "ph reset") {
       // 恢复默认开关连接
       AppPHCfg.DswitchSel = SWD_OPEN;
       AppPHCfg.PswitchSel = SWP_PL|SWP_PL2;
       AppPHCfg.NswitchSel = SWN_NL|SWN_NL2;
       AppPHCfg.TswitchSel = SWT_AIN0 | SWT_TRTIA; 
       AppPHCfg.bParaChanged = bTRUE;
       AppPHInit(AppBuff, APPBUFF_SIZE);
       Serial.println("Restored to Normal pH Mode.");
    } 
    else if (cmd.startsWith("ph cal gain")) {
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
                    g_isCalibratingGain = true;
                    g_isPhMode = false; // 暂停普通打印
                    g_isCalibratingOffset = false;
                    AppPHCtrl(PHCTRL_START, 0);
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

  }
}