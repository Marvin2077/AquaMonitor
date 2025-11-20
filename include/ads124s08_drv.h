#pragma once
#include <stdint.h>
#include <stddef.h>
#include <Arduino.h>

/**
 * ADS124S08 minimal SPI driver
 * - 软复位 / START / STOP
 * - 读写寄存器（RREG/WREG）
 * - DRDY 等待（GPIO 或简单延时）
 * - 读取 24-bit 数据（并做符号扩展）
 * - 默认配置：差分 AIN0-AIN1、内部2.5V参考、20SPS、连续转换
 *
 * 使用说明：
 *  1) 构造时传入 HAL 回调（cs_assert/cs_release/spi_txrx/delay/drdy）
 *  2) softReset() → defaultConfig() → start() → waitDRDY() → readData()
 */
class ADS124S08_Drv {
public:
  // ===== SPI 命令 =====
  enum class Cmd : uint8_t {
    NOP   = 0x00,
    RESET = 0x06,
    START = 0x08,
    STOP  = 0x0A,
    RDATA = 0x12,
    RREG  = 0x20,   // 实际发送: (RREG | (addr & 0x1F)), 再发 (num-1)
    WREG  = 0x40    // 同上
  };

  // ===== 常用寄存器地址（子集）=====
  enum class Reg : uint8_t {
    ID        = 0x00,
    STAT      = 0x01,
    INPMUX    = 0x02,   // 输入多路复用
    PGA_GAIN  = 0x03,   // PGA/使能等
    DATARATE  = 0x04,   // 模式/数据率
    REF       = 0x05,   // 参考配置
    IDACMAG   = 0x06,
    IDACMUX   = 0x07,
    VBIAS     = 0x08,
    SYS       = 0x09,   // SENDSTAT/CRC等
    OFCAL0    = 0x0A,
    OFCAL1    = 0x0B,
    OFCAL2    = 0x0C,
    FSCAL0    = 0x0D,
    FSCAL1    = 0x0E,
    FSCAL2    = 0x0F
  };

  // ===== DATARATE(0x04) 位（按需增补）=====
  struct DR04 {
    static constexpr uint8_t DR_MASK      = 0x0F;   // DR[3:0]
    static constexpr uint8_t MODE_SINGLE  = 0x10;   // 假定 bit4：1=single, 0=cont
    static constexpr uint8_t MODE_CONT    = 0x00;
    static constexpr uint8_t DR_20SPS     = 0x04;   // 示例档位（可按手册补全其他）
  };

  // ===== REF(0x05) 位（按需增补）=====
  struct REF05 {
    static constexpr uint8_t REFSEL_MASK  = 0b00001100;
    static constexpr uint8_t REFSEL_INT25 = 0b00001000; // 内部2.5V参考
    static constexpr uint8_t REFCON_MASK  = 0b00000011;
    static constexpr uint8_t REFCON_AON   = 0b10;       // 内参始终开启
    // 如需调整缓冲/外参，可继续补
  };

  // ===== SYS(0x09) 位（按需增补）=====
  struct SYS09 {
    static constexpr uint8_t SENDSTAT = 0b00000001;
    static constexpr uint8_t CRC      = 0b00000010;
  };

  // ===== HAL 回调 =====
struct Hal {
  void (*cs_assert)();
  void (*cs_release)();
  bool (*spi_txrx)(const uint8_t*, uint8_t*, size_t);
  void (*delay_us)(uint32_t);
  void (*delay_ms)(uint32_t);
  int  (*read_drdy)();
};

  explicit ADS124S08_Drv(const Hal& hal) : hal_(hal) {}

  // 基本命令
  bool softReset();
  bool start();
  bool stop();

  // 读写寄存器
  bool readRegisters(uint8_t addr, uint8_t* buf, size_t n);
  bool writeRegisters(uint8_t addr, const uint8_t* data, size_t n);
  bool writeRegisterMasked(uint8_t addr, uint8_t mask, uint8_t value);
  // 读设备ID
  bool readID(uint8_t& id);
  //翻转寄存器中的某一位
  bool toggleBit(uint8_t addr, uint8_t bitPos);
  
  // 数据路径
  bool waitDRDY(uint32_t timeout_ms);
  bool readDataRaw(uint32_t& raw24);     // 只返回 24 位原始值（未开 STATUS/CRC）
  bool readData(int32_t& value24_signed);// 返回 24→32 位符号扩展后的值

  // 默认配置（示例：差分 AIN0-AIN1、内部2.5V参考、20SPS、连续）
  bool defaultConfig();
  // PT1000测温
  bool readPT1000Temperature(float r_ref, float& temperature);
  // 工具：24 位补码 → int32
  static inline int32_t signExtend24(uint32_t raw24) {
    return (raw24 & 0x800000) ? int32_t(raw24 | 0xFF000000) : int32_t(raw24 & 0x00FFFFFF);
  }

  // 工具：码值→电压（双极性满量程假定 2^23）
  static inline double codeToVolt(int32_t code24, double vref, int gain) {
    constexpr double FS = 8388608.0; // 2^23
    return (double)code24 / FS * (vref / (double)gain);
  }

private:
  bool sendCmd(uint8_t cmd);

  Hal hal_;
};
//装配成回调是什么意思 就是怎么通过回调完成你说的片选收发等，能结合具体代码给我讲一下吗 逻辑我能明白 但是具体的代码实现我还是没看懂
/*
工具函数是什么 一般写法是什么？为什么要这样写？好处是什么？
私有函数是什么意思，如果要使用私有函数或者私有成员应该如何访问呢？
构造函数是什么？怎么起作用的？
*/