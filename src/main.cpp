#include <Arduino.h>
#include <SPI.h>

// 包含您编写的所有驱动层头文件
#include "spi_hal.h"        // 底层硬件操作
#include "ads124s08_board_glue.h"     // 胶水层
#include "ads124s08_drv.h"  // 通用驱动层
#include "ad5941_board_glue.h"
#include "ad5941_drv.h"
#include "cond_service.h"

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

// 定义外部参考电压值，用于后续电压转换计算
// 请根据您实际连接到 REFP0/REFN0 的参考电压修改此值
const double V_REF = 1.57; 

// 这里先声明，具体初始化在 setup() 中完成
// 创建一个 ADS124S08 驱动对象（此时还未初始化）
ADS124S08_Drv* adc = nullptr;
// 创建 AD5941 驱动对象指针
AD5941_Drv* afe = nullptr;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("--- ADS124S08 PT1000 Demo ---");
  Serial.printf("Using theoretical V_REF = %.2fV\n", V_REF);
  SpiHAL::beginBus(PIN_SCLK, PIN_MISO, PIN_MOSI);
  Serial.println("SPI bus initialized.");

// --- 初始化 ADS124S08 ---
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
  
}

void loop() {
  if (!adc) {
    return;
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
      
    } else {
      Serial.println("Failed to read data from ADS124S08.");
    }

  } else {
    Serial.println("Timeout: DRDY pin did not go low.");
  }

  delay(500);
}