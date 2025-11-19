#include <Arduino.h>
#include <SPI.h>

//所有驱动层头文件
#include "spi_hal.h"        // 底层硬件操作
#include "ads124s08_board_glue.h"     // 胶水层
#include "ads124s08_drv.h"  // 通用驱动层
#include "ad5941_board_glue.h"      // 引入 Glue 层头文件
#include "ad5941PlatformCfg.h"
#include "temp_service.h" 
#include "impedance_service.h" 
#include "storage_manager.h"
#include "mux_iface.h"
extern "C" {
#include "ad5940.h"
}

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
// CHANNEL_MUX_ADDR1 复位引脚 GPIO-18
// CHANNEL_MUX_ADDR0 复位引脚 GPIO-17
// ISFET_MUX_ADDR2引脚 GPIO-25
// ISFET_MUX_ADDR1引脚 GPIO-33
// ISFET_MUX_ADDR0引脚 GPIO-32

// 定义外部参考电压值，用于后续电压转换计算 请根据您实际连接到 REFP0/REFN0 的参考电压修改此值
const double V_REF = 1.68; 

// === 全局变量 ===
uint32_t impDataBuffer[512]; // 用于存放阻抗数据

// 创建一个 ADS124S08 驱动对象（此时还未初始化）
ADS124S08_Drv* ads124s08 = nullptr;
//创建 Temp Service 对象指针
TempService* g_tempSvc = nullptr;

// === Calibration control ===
static bool cal_enabled = false;   // 默认不进校准流程
// 可选：启动就想进校准可改为 true
extern void handleCondCommand(const String& line); // 前置声明


void setup() {
  Serial.begin(115200);
  delay(1000);
  // 初始化测量通道MUX的地址引脚
  // 初始化为通道1
  pinMode(CHANNEL_MUX_ADDR1, OUTPUT);
  digitalWrite(CHANNEL_MUX_ADDR1, HIGH);
  pinMode(CHANNEL_MUX_ADDR0, OUTPUT);
  digitalWrite(CHANNEL_MUX_ADDR0, LOW);
  pinMode(ISFET_MUX_ADDR2, OUTPUT);
  digitalWrite(ISFET_MUX_ADDR2, LOW);
  pinMode(ISFET_MUX_ADDR1, OUTPUT);
  digitalWrite(ISFET_MUX_ADDR1, LOW);
  pinMode(ISFET_MUX_ADDR0, OUTPUT);
  digitalWrite(ISFET_MUX_ADDR0, LOW);

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

  // 2) 配置并初始化 Ad5941 Glue 层
  
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
  Serial.println("ADS124S08 configuration successful.");

  Serial.println("ADC started. Waiting for data...");
  Serial.println("----------------------------------------");
  // --- 完成ADS124S08的初始化 --- //
  //完成芯片初始化
  Serial.println("System Initialized");
    
  // 测试MUX控制
  ChooseSenesingChannel(1);
  ChooseISFETChannel(2);
}


void loop() {


}

