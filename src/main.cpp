#include <Arduino.h>
#include <SPI.h>

#include "storage_manager.h"
#include "mux_iface.h"
#include "spi_hal.h"        
// 芯片驱动与 Glue 头文件
#include "ads124s08_board_glue.h"    
#include "ads124s08_drv.h"  
extern "C" {
#include "ad5940.h"
}
#include "ad5941_board_glue.h"   
#include "ad5941PlatformCfg.h"
// 服务头文件
#include "temp_service.h" 
#include "Conductivity_service.h"
#include "ph_service.h"
//指令解析与发送头文件
#include "app_globals.h"
#include "command_parser.h"
#include "command_dispatcher.h"

/* ====== 共用 SPI 总线引脚 ====== */
static const int PIN_SCLK = 14; // ESP32 SPI 时钟引脚
static const int PIN_MISO = 12; // ESP32 SPI MISO 引脚
static const int PIN_MOSI = 13; // ESP32 SPI MOSI 引脚
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
float g_condCalStdValue = 0.0f;  // 三点校准当前点标准液值
int g_condCalSlot = 0;           // 下一个要记录的校准点槽位(0/1/2)
bool g_isCalibratingCond = false;
// === pH 服务标志位/全局变量 ===
bool g_ispHMode = false;
bool g_isCalibratingOffset = false; // 是否正在进行 Offset 校准
bool g_isCalibratingGain = false;   // 是否正在校准增益
float g_calResistorValue = 1000.0f; // 用户输入的校准电阻阻值(默认 1k)
uint8_t g_isfetChannel = 1;   // 默认 ISFET 通道

// === ADS124S08 全局变量 ===
// 创建一个 ADS124S08 驱动对象（此时还未初始化）
ADS124S08_Drv* ads124s08 = nullptr;
// === Temperature 服务标志位/全局变量 ===
TempService* g_tempSvc = nullptr;
const double MY_RREF = 3300.0;
bool is_calibrating = false;
double currentTemp = 0.0;
bool g_isTempMode = false;

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

  // 初始化 SPI 总线
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
  Serial.println(" ADS124S08 Setup Start");
  Serial.println("======================================");
   // 1) 初始化 ADS124S08 SPI 总线
  static SpiDevice ads_spidev(CS_ADS124S08, 4000000, MSBFIRST, SPI_MODE1);
  Serial.println("SpiDevice for AD5941 created.");
  // 2) 定义 ADS124S08 的 Glue 配置
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

  // 3. 读取并应用温度参数
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
  // 4. 读取并应用电导率三点校准参数
  CondCalibData condCalData = loadCondCalib();
  if (condCalData.valid) {
    g_condCalib.a = condCalData.a;
    g_condCalib.b = condCalData.b;
    g_condCalib.c = condCalData.c;
    g_condCalib.valid = true;
    Serial.println("[Setup] Loaded Cond 3-Point Calibration.");
  } else {
    Serial.println("[Setup] No valid Cond 3-Point Calibration found.");
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
      Serial.printf("$TEMP,MEAS,%.3f*\n", currentTemp);
      currentState = STATE_IDLE;
      break;
    case STATE_TEMP_CAL_P1:
      if (g_tempSvc->recordCalibPoint(0, 25.0)) 
      {
         Serial.println("$TEMP,CAL_PT,1,25.0,OK*");
         currentState = STATE_IDLE;     
      } else 
      {
         Serial.println("$ERR,TEMP,Point 1 Failed*");
         currentState = STATE_IDLE;     
      }
      currentState = STATE_IDLE;
      break;
    case STATE_TEMP_CAL_P2:
      if (g_tempSvc->recordCalibPoint(1, 35.0)) 
      {
         Serial.println("$TEMP,CAL_PT,2,35.0,OK*");
         currentState = STATE_IDLE;
      } else 
      {
         Serial.println("Point 2 Failed!");
         currentState = STATE_IDLE;         
      }
      break;
    case STATE_TEMP_CAL_P3:
      if (g_tempSvc->recordCalibPoint(2, 50.0)) 
      {
         Serial.println("Point 3 Saved!");
         currentState = STATE_IDLE;
      } else 
      {
         Serial.println("Point 3 Failed!");
         currentState = STATE_IDLE;
      }
 
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
        TempService::CalibCoeff clean; // 默认 valid=false
        g_tempSvc->setCalib(clean);
        Serial.println("Calibration Cleared.");
        currentState = STATE_IDLE;
      }
    break;
    case STATE_TEMP_RESISTANCE:
    {
      double r_ohm = 0.0;
      if (g_tempSvc->readResistance(r_ohm)) {
        Serial.printf("$TEMP,RES,%.2f*\n", r_ohm);
      } else {
        Serial.println("[ERR] Resistance read failed");
      }
      currentState = STATE_IDLE;
      break;
    }
    // === 处理电导率服务 ===
    case STATE_COND_INIT:
      g_isCondMode = true;
      g_ispHMode = false;
      ChooseSenesingChannel(1);
      if(AppCondInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK)
      {
        AppPHCfg.PHInited = bFALSE;
        Serial.println("$COND,INIT,OK*");
      }
      else
      {
        Serial.println("$ERR,COND,Init failed*");
        currentState = STATE_IDLE;
        break;
      }
      currentState = STATE_IDLE;
    break;
    case STATE_COND_MEASURE:
      if(AppCondCfg.CondInited == bFALSE || g_isCondMode == false)
      {
        Serial.println("$ERR,COND,Not initialized*");
        currentState = STATE_IDLE;
        break;
      }
      if (AppCondISR(AppBuff, &tempCount) == 0)
      {
        if (tempCount > 0)
        {
          CondShowResult(AppBuff, tempCount, false, 0, 0);
          currentState = STATE_IDLE;
        }
      }
    break;
    case STATE_COND_SWEEP:
      if(AppCondCfg.CondInited == bFALSE || g_isCondMode == false)
      {
        g_isSweepMode = false;
        g_sweepCount  = 0;
        AppCondCfg.SweepCfg.SweepEn    = bFALSE;
        AppCondCfg.SweepCfg.SweepIndex = 0;
        Serial.println("$ERR,COND,Not initialized*");
        currentState = STATE_IDLE;
        break;
      }
      if(AppCondISR(AppBuff, &tempCount) == 0)
      {
        if(tempCount > 0)
        {
          CondShowResult(AppBuff, tempCount, true, g_sweepCount + 1, g_sweepTotalPoints);
          g_sweepCount++;
          if (g_sweepCount < g_sweepTotalPoints)
            {
              AppCondCtrl(CondCTRL_START, 0);
            }
          else
            {
              Serial.println("$COND,SWEEP_END*");
              g_isSweepMode = false;
              g_sweepCount  = 0;  
              AppCondCfg.SweepCfg.SweepIndex = 0;
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
    Serial.println("$ERR,COND,Not initialized*");
    currentState = STATE_IDLE;
    break;
  }

  if(AppCondISR(AppBuff, &tempCount) == 0)
  {
    if(tempCount > 0)
    {
      static uint8_t calSampleCount = 0;
      static float   calAccumG      = 0.0f;
      static float   calAccumT      = 0.0f;
      const  uint8_t CAL_SAMPLES    = 6;

      float measured_G = ComputeKCell(AppBuff, tempCount);
      if(measured_G <= 0.0f)
      {
        Serial.println("$ERR,COND,CAL,Invalid G=0*");
        calSampleCount = 0; calAccumG = 0.0f; calAccumT = 0.0f;
        currentState = STATE_IDLE;
        break;
      }

      g_tempSvc->measure(currentTemp);
      calAccumG += measured_G;
      calAccumT += currentTemp;
      calSampleCount++;

      Serial.printf("$COND,CAL,SAMPLE,%d/%d,G=%.4f,T=%.2f*\n",
                    calSampleCount, CAL_SAMPLES, measured_G, currentTemp);

      if(calSampleCount < CAL_SAMPLES)
      {
        AppCondCtrl(CondCTRL_START, 0);  // 继续触发下一次测量
      }
      else
      {
        float avg_G = calAccumG / (float)CAL_SAMPLES;
        float avg_T = calAccumT / (float)CAL_SAMPLES;
        float ec_true = ApecaStd1413_TrueEC(avg_T);
        float new_K   = ec_true / avg_G;

        AppCondCfg.K_Cell = new_K;
        saveCondParams(new_K);
        // $COND,CAL,DONE,<avg_G>,<ec_true>,<avg_T>,<new_K>*
        Serial.printf("$COND,CAL,DONE,%.4f,%.2f,%.2f,%.4f*\n",
                      avg_G, ec_true, avg_T, new_K);

        calSampleCount = 0; calAccumG = 0.0f; calAccumT = 0.0f;
        currentState = STATE_IDLE;
      }
    }
  }
break;
    case STATE_COND_CAL_P1:
    case STATE_COND_CAL_P2:
    case STATE_COND_CAL_P3:
    {
      static uint8_t sampleCount = 0;
      static float   accum       = 0.0f;
      const  uint8_t SAMPLES     = 10;
      if (AppCondCfg.CondInited == bFALSE || g_isCondMode == false) {
        Serial.println("$ERR,COND,Not initialized*");
        sampleCount = 0; accum = 0.0f;
        currentState = STATE_IDLE;
        break;
      }
      if (AppCondISR(AppBuff, &tempCount) == 0) {
        if (tempCount > 0) {
          float raw_G = ComputeKCell(AppBuff, tempCount);
          if(raw_G <= 0.0f)
          {
            Serial.println("$ERR,COND,CAL_P,Invalid G=0, retrying*");
            AppCondCtrl(CondCTRL_START, 0);  // 重触发，不累加，不计数
            break;
          }
          float rawCond = raw_G * AppCondCfg.K_Cell;
          accum += rawCond;
          sampleCount++;
          if (sampleCount < SAMPLES) {
            AppCondCtrl(CondCTRL_START, 0);
          } else {
            float avgCond = accum / (float)SAMPLES;
            g_condCalPoints[g_condCalSlot].cond_true = g_condCalStdValue;
            g_condCalPoints[g_condCalSlot].cond_meas = avgCond;
            g_condCalPoints[g_condCalSlot].recorded  = true;
            Serial.printf("$COND,CAL_PT,%d,%.2f,%.4f*\n",
                          g_condCalSlot, g_condCalStdValue, avgCond);
            g_condCalSlot++;
            sampleCount = 0;
            accum = 0.0f;
            currentState = STATE_IDLE;
          }
        }
      }
    }
    break;
    case STATE_COND_SAVE_CAL:
    {
      if (!g_condCalPoints[0].recorded || !g_condCalPoints[1].recorded || !g_condCalPoints[2].recorded) {
        int n = (int)g_condCalPoints[0].recorded + (int)g_condCalPoints[1].recorded + (int)g_condCalPoints[2].recorded;
        Serial.printf("$ERR,COND,Not all 3 points recorded (%d/3)*\n", n);
        currentState = STATE_IDLE;
        break;
      }
      if (CondFitThreePoint(g_condCalPoints[0], g_condCalPoints[1], g_condCalPoints[2], g_condCalib)) {
        saveCondCalib(g_condCalib.a, g_condCalib.b, g_condCalib.c, true);
        Serial.printf("$COND,CAL_SAVE,%.6f,%.6f,%.6f*\n",
                      g_condCalib.a, g_condCalib.b, g_condCalib.c);
      } else {
        Serial.println("$ERR,COND,3-point fit failed*");
      }
      currentState = STATE_IDLE;
    }
    break;
    case STATE_COND_RESET_CAL:
    {
      g_condCalib = CondCalibCoeff{};
      saveCondCalib(0.0f, 1.0f, 0.0f, false);
      for (int i = 0; i < 3; i++) {
        g_condCalPoints[i].recorded  = false;
        g_condCalPoints[i].cond_true = 0.0f;
        g_condCalPoints[i].cond_meas = 0.0f;
      }
      g_condCalSlot = 0;
      Serial.println("$COND,CAL_RESET,OK*");
      currentState = STATE_IDLE;
    }
    break;
    // === 处理pH服务 ===
    case STATE_PH_INIT:
      ChooseSenesingChannel(3);
      ChooseISFETChannel(g_isfetChannel);
      g_isCondMode = false;
      g_ispHMode = true;
      if(AppPHInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK) 
      {
        AppCondCfg.CondInited = bFALSE;
        Serial.println("$PH,INIT,OK*");
      }
      else 
      {
        Serial.println("$ERR,PH,INIT_FAILED*");
        currentState = STATE_IDLE;
        break;
      } 
      currentState = STATE_IDLE;
    break;
    case STATE_PH_MEASURE:
      if(AppPHCfg.PHInited == bFALSE || g_ispHMode == false )
      {
        Serial.println("$ERR,PH,NOT_INITIALIZED*");
        currentState = STATE_IDLE;
        break;
      }
      if (AppPHISR(AppBuff, &tempCount) == 0) 
      {
        if (tempCount > 0) 
        {
          PHShowResult(AppBuff, tempCount);
          Serial.printf("$PH,MEAS_DONE,%lu*\n", (unsigned long)tempCount);
          currentState = STATE_IDLE;
        }
      }
    break;
    case STATE_PH_CAL_OFFSET:
      if(AppPHCfg.PHInited == bFALSE || g_ispHMode == false )
      {
        Serial.println("$ERR,PH,NOT_INITIALIZED*");
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
          Serial.printf("$PH,CAL_OFFSET,OK,%u*\n", (unsigned)measured_offset);
          // 校准完成后，自动恢复开关设置到正常模式
          AppPHCfg.TswitchSel = SWT_AIN1 | SWT_TRTIA; // 恢复连接传感器
          AppPHCfg.bParaChanged = bTRUE;
          AppPHInit(AppBuff, APPBUFF_SIZE); // 重建序列
          currentState = STATE_IDLE;
        }
      }
    break;
    case STATE_PH_CAL_GAIN:
      if(AppPHCfg.PHInited == bFALSE || g_ispHMode == false )
      {
        Serial.println("$ERR,PH,NOT_INITIALIZED*");
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
          // 所以 R_tia_real = (|Volt| * R_ext) / 1.1V
          if (abs_volt > 0.05f) // 确保有足够电压，防止除零或噪声干扰
          { 
            float calculated_rtia = (abs_volt * g_calResistorValue) / 1.1f;
            // 4. 更新系统参数
            AppPHCfg.Rtia_Value_Ohm = calculated_rtia;
            savePhParams(AppPHCfg.ZeroOffset_Code, calculated_rtia);
            Serial.printf("$PH,CAL_GAIN,OK,%.2f,%.1f,%u,%.6f*\n",
                          calculated_rtia, g_calResistorValue, (unsigned)rawCode, voltage_diff);
            currentState = STATE_IDLE;
          }
          else 
          {
            Serial.printf("$ERR,PH,SIGNAL_LOW,%.6f*\n", abs_volt);
            currentState = STATE_IDLE;
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


    ParsedCommand parsed = parseCommand(cmd);
    if (parsed.valid) {
          dispatchCommand(parsed);
          return;
    }
    Serial.println("$ERR,UNKNOWN_CMD*");
  }
}
