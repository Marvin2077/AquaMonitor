#include "ads124s08_board_glue.h"
#include "ads124s08_drv.h"
// ---- 私有：保存 ADS124S08 的板级状态，让回调能访问 ----
/*

g (全局结构体) 
  ↓
g_ads.spi (指向SpiDevice的指针)
  ↓
g_ads.spi->beginTxn() (调用SpiDevice的成员函数)
  ↓
SPI.beginTransaction(settings) (最终调用Arduino SPI库)

*/
namespace {
  struct AdsState {
    const SpiDevice* dev = nullptr;
    uint8_t pin_drdy = 0xFF;
    uint8_t pin_reset = 0xFF;
  } g_ads;

  // 回调实现（驱动会调用这些）
  void cs_assert()               { g_ads.dev->cs_low(); }
  void cs_release()              { g_ads.dev->cs_high(); }
  bool spi_txrx(const uint8_t* tx, uint8_t* rx, size_t n) {
    g_ads.dev->beginTxn();
    g_ads.dev->transfer(tx, rx, n);
    g_ads.dev->endTxn();
    return true;
  }
  void delay_us_(uint32_t us)    { SpiHAL::delay_us(us); }
  void delay_ms_(uint32_t ms)    { SpiHAL::delay_ms(ms); }
  int  read_drdy_()              { return digitalRead(g_ads.pin_drdy); } // 0=ready
}

// ---- 对外 API 实现 ----
ADS124S08_Drv::Hal BoardGlue::make_ads_hal(const AdsGlueConfig& cfg) {
  // 1) 保存板级状态 (这部分不变)
  g_ads.dev       = cfg.spi;
  g_ads.pin_drdy  = cfg.pin_drdy;
  g_ads.pin_reset = cfg.pin_reset;

  // 2) 初始化相关 GPIO (这部分不变)
  g_ads.dev->begin();
  pinMode(g_ads.pin_drdy, INPUT);
  if (g_ads.pin_reset != 0xFF) { // 检查引脚是否有效
    pinMode(g_ads.pin_reset, OUTPUT);
    digitalWrite(g_ads.pin_reset, HIGH);
  }

  // 3) 返回驱动 HAL (使用 C++11 兼容的顺序初始化语法)
  ADS124S08_Drv::Hal hal {
    cs_assert,   // 对应 1. cs_assert
    cs_release,  // 对应 2. cs_release
    spi_txrx,    // 对应 3. spi_txrx
    delay_us_,   // 对应 4. delay_us
    delay_ms_,   // 对应 5. delay_ms
    read_drdy_   // 对应 6. read_drdy
  };
  return hal;
}

void BoardGlue::ads_hw_reset(const AdsGlueConfig& cfg) {
  // 低脉冲 >= 4*tCLK，随后等待 td(RSSC)；保守给 10us + 2ms
  digitalWrite(cfg.pin_reset, LOW);
  SpiHAL::delay_us(10);
  digitalWrite(cfg.pin_reset, HIGH);
  SpiHAL::delay_ms(2);
}
