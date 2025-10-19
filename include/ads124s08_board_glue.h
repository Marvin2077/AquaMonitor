#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "spi_hal.h"
#include "ads124s08_drv.h"
// 如需给 AD5941 也做 glue，包含它的 drv 头文件：#include "ad5941_drv.h"

// ====== ADS124S08 glue 对外配置 ======
struct AdsGlueConfig {
  const SpiDevice* spi;    // 哪条 SPI+CS 以及 MODE/速率
  uint8_t pin_drdy;        // DRDY（低有效）
  uint8_t pin_reset;       // RESET（低有效或低脉冲，按你板卡连法）
};

// ====== 对外 API ======
namespace BoardGlue {

  // 绑定 glue，并产出一份可交给驱动的 HAL（函数指针都指向 glue）
  ADS124S08_Drv::Hal make_ads_hal(const AdsGlueConfig& cfg);

  // 可选：硬件复位（便于 main 简洁调用）
  void ads_hw_reset(const AdsGlueConfig& cfg);

  // 若你后续也做 AD5941 glue，可以仿照上面再给一套：
  // Ad5941GlueConfig/make_ad5941_hal()/ad5941_hw_reset() ...
}
