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
#include "state_processors.h"

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
    case STATE_IDLE: break;
    case STATE_TEMP_MEASURE:     processTempMeasure();    break;
    case STATE_TEMP_CAL_P1:      processTempCalP1();      break;
    case STATE_TEMP_CAL_P2:      processTempCalP2();      break;
    case STATE_TEMP_CAL_P3:      processTempCalP3();      break;
    case STATE_TEMP_SAVE_CAL:    processTempSaveCal();    break;
    case STATE_TEMP_RESET_CAL:   processTempResetCal();   break;
    case STATE_TEMP_RESISTANCE:  processTempResistance(); break;
    case STATE_COND_INIT:        processCondInit();       break;
    case STATE_COND_MEASURE:     processCondMeasure();    break;
    case STATE_COND_SWEEP:       processCondSweep();      break;
    case STATE_PH_INIT:          processPhInit();         break;
    case STATE_PH_MEASURE:       processPhMeasure();      break;
    case STATE_PH_CAL_OFFSET:    processPhCalOffset();    break;
    case STATE_PH_CAL_GAIN:      processPhCalGain();      break;
    case STATE_PH_CHANNEL:       processPhChannel();      break;
    default: break;
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
