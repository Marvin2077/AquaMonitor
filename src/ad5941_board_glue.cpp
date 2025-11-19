#include "ad5941_board_glue.h"
/*

g (全局结构体) 
  ↓
g_ad5941.spi (指向SpiDevice的指针)
  ↓
g_ad5941.spi->beginTxn() (调用SpiDevice的成员函数)
  ↓
SPI.beginTransaction(settings) (最终调用Arduino SPI库)

*/
extern "C" {
#include "ad5940.h"
}
// ---- 私有：保存 AD5941 的板级状态，让回调能访问 ----
namespace {

struct State {
  const SpiDevice* spi = nullptr;
  uint8_t pin_reset = 0xFF;
  uint8_t pin_int   = 0xFF;
  bool in_txn = false;  // 是否处于一次 SPI 事务中（CsClr..CsSet 之间）
} g_ad5941;

// ---- 以下为 ADI 库需要的 C 回调 ----
extern "C" void AD5940_CsClr(void) {
  if (!g_ad5941.spi) return;
  g_ad5941.spi->beginTxn();         // 进入一次事务
  g_ad5941.in_txn = true;
  g_ad5941.spi->cs_low();           // 片选有效
}

extern "C" void AD5940_CsSet(void) {
  if (!g_ad5941.spi) return;
  g_ad5941.spi->cs_high();          // 片选无效
  if (g_ad5941.in_txn) {            // 结束事务
    g_ad5941.spi->endTxn();
    g_ad5941.in_txn = false;
  }
}

extern "C" void AD5940_ReadWriteNBytes(unsigned char *pSendBuffer, unsigned char *pRecvBuff, unsigned long length) {
  if (!g_ad5941.spi || length == 0) return;
  g_ad5941.spi->transfer(pSendBuffer, pRecvBuff, static_cast<size_t>(length));
}

extern "C" void AD5940_Delay10us(uint32_t n) {
  // ADI 要求：单位为 10us
  if (n == 0) return;
  SpiHAL::delay_us(n * 10);
}

// 若接了硬复位脚，可以用库的复位钩子；未配置则空操作
extern "C" void AD5940_RstClr(void) {
  if (g_ad5941.pin_reset == 0xFF) return;
  digitalWrite(g_ad5941.pin_reset, LOW);
}
extern "C" void AD5940_RstSet(void) {
  if (g_ad5941.pin_reset == 0xFF) return;
  digitalWrite(g_ad5941.pin_reset, HIGH);
}

/* ================ 新增存根函数 ================ */

/**
 * @brief 清除MCU的中断标志 (存根)
 * @details 我们的设计使用轮询，不使用MCU中断, 故此函数为空。
 * 但库函数(AppBIOZInit)需要它, 必须定义。
 */
extern "C" uint32_t AD5940_ClrMCUIntFlag(void) {
  // 我们不使用MCU中断, 所以这里什么都不做
  return 1;
}

/**
 * @brief 获取MCU的中断标志 (存根)
 * @details 我们的设计使用轮询, 不使用MCU中断。
 * 为防止将来其他函数调用它, 我们提供一个存根。
 */
extern "C" uint32_t AD5940_GetMCUIntFlag(void) {
  // 我们不使用MCU中断, 总是返回0 (没有中断)
  return 0;
}

} // anonymous namespace

// ---- 对外 API 实现 ----
namespace Ad5941Glue {

void setup(const Config& cfg) {
  g_ad5941.spi       = cfg.spi;
  g_ad5941.pin_reset = cfg.pin_reset;
  g_ad5941.pin_int   = cfg.pin_int;

  if (g_ad5941.spi) {
    // 初始化 CS 引脚并默认拉高
    g_ad5941.spi->begin();
  }
  if (g_ad5941.pin_reset != 0xFF) {
    pinMode(g_ad5941.pin_reset, OUTPUT);
    digitalWrite(g_ad5941.pin_reset, HIGH);
  }
  if (g_ad5941.pin_int != 0xFF) {
    pinMode(g_ad5941.pin_int, INPUT_PULLUP);
  }
}

void hardware_reset(uint16_t low_us, uint16_t settle_ms) {
  if (g_ad5941.pin_reset == 0xFF) return;
  digitalWrite(g_ad5941.pin_reset, LOW);
  SpiHAL::delay_us(low_us);
  digitalWrite(g_ad5941.pin_reset, HIGH);
  SpiHAL::delay_ms(settle_ms);
}

uint32_t read_chip_id() {
  // 可在 AD5940_Initialize() 前后调用，作为通信冒烟测试
  return AD5940_ReadReg(REG_AFECON_ADIID);
}

bool ready() {
  return g_ad5941.spi != nullptr;
}

} // namespace Ad5941Glue
