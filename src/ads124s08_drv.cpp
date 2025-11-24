#include "ads124s08_drv.h"
#include "math.h"
// ===== 内部工具 =====
bool ADS124S08_Drv::sendCmd(uint8_t cmd) {
  if (!hal_.cs_assert || !hal_.cs_release || !hal_.spi_txrx) return false;
  hal_.cs_assert();
  bool ok = hal_.spi_txrx(&cmd, nullptr, 1);
  hal_.cs_release();
  if (hal_.delay_us) hal_.delay_us(50);
  return ok;
}

// ===== 基本命令 =====
bool ADS124S08_Drv::softReset() { return sendCmd(static_cast<uint8_t>(Cmd::RESET)); }
bool ADS124S08_Drv::start()     { return sendCmd(static_cast<uint8_t>(Cmd::START)); }
bool ADS124S08_Drv::stop()      { return sendCmd(static_cast<uint8_t>(Cmd::STOP));  }

// ===== 读写寄存器 =====
// 组装命令帧+SPI发送命令+读回寄存器内容
bool ADS124S08_Drv::readRegisters(uint8_t addr, uint8_t* buf, size_t n) {
  if (!buf || n == 0) return false; //参数合法性检查 buf/n任意为0则返回失败
  uint8_t head[2] = {
    static_cast<uint8_t>(static_cast<uint8_t>(Cmd::RREG) | (addr & 0x1F)),
    //内层的或运算会把结果提升为int类型，通常为32位，因此外层再转换一次，将结果变回8位的范围
    static_cast<uint8_t>(n - 1)
  };
  hal_.cs_assert();
  if (!hal_.spi_txrx(head, nullptr, 2)) { hal_.cs_release(); return false; }
  bool ok = hal_.spi_txrx(nullptr, buf, n);
  hal_.cs_release();
  return ok;
}
//writeRegister会修改整个字节，如果想单独修改字节中的某一位，则需要用writeRegisterMasked
bool ADS124S08_Drv::writeRegisters(uint8_t addr, const uint8_t* data, size_t n) {
  if (!data || n == 0) return false;
  uint8_t head[2] = {
    static_cast<uint8_t>(static_cast<uint8_t>(Cmd::WREG) | (addr & 0x1F)),
    static_cast<uint8_t>(n - 1)
  };
  hal_.cs_assert();
  if (!hal_.spi_txrx(head, nullptr, 2)) { hal_.cs_release(); return false; }
  bool ok = hal_.spi_txrx(data, nullptr, n);
  hal_.cs_release();
  if (hal_.delay_us) hal_.delay_us(50);
  return ok;
} 
// 读-改-写
bool ADS124S08_Drv::writeRegisterMasked(uint8_t addr, uint8_t mask, uint8_t value) {
  uint8_t cur = 0;
  if (!readRegisters(addr, &cur, 1)) return false;//读取目标寄存器的值并将值放入cur中
  cur = (cur & ~mask) | (value & mask);//先将要修改的位清零，再取出新值，最后用或操作合并
  return writeRegisters(addr, &cur, 1);
}

// ===== 读ID =====
bool ADS124S08_Drv::readID(uint8_t& id)
{
  return readRegisters(static_cast<uint8_t>(Reg::ID),&id,1);
  
}
// =====翻转寄存器某位
bool ADS124S08_Drv::toggleBit(uint8_t addr, uint8_t bitPos)
{
    if(bitPos>7)
    {
      return false;
    } 
    uint8_t cur = 0;
    if(!readRegisters(addr,&cur,1)) return false;
    cur ^= (uint8_t(1u)<< bitPos);
    return writeRegisters(addr,&cur,1);
  }
// ===== DRDY 等待 =====
bool ADS124S08_Drv::waitDRDY(uint32_t timeout_ms) {
  if (!hal_.read_drdy) { // 如果没有提供 DRDY 回调，则简单延时
    if (hal_.delay_ms) hal_.delay_ms(55); // 20SPS 对应 50ms 周期，多等一点
    return true;
  }

  // 直接等待 DRDY 变低即可
  const uint32_t start_time = millis();
  while (hal_.read_drdy() != 0) { // DRDY 是低电平有效，所以等待它不为高
    if (millis() - start_time > timeout_ms) {
      return false; // 超时
    }
    yield(); // 在长等待中让出CPU给其他任务，对ESP32很重要
  }
  return true; // 成功等到低电平
}


// ===== 读取 24 位数据 =====
bool ADS124S08_Drv::readDataRaw(uint32_t& raw24) {
  uint8_t cmd = static_cast<uint8_t>(Cmd::RDATA);
  uint8_t b[3] = {0};
  hal_.cs_assert();
  if (!hal_.spi_txrx(&cmd, nullptr, 1)) { hal_.cs_release(); return false; }
  if (hal_.delay_us) hal_.delay_us(10);
  bool ok = hal_.spi_txrx(nullptr, b, 3);
  hal_.cs_release();
  if (!ok) return false;
  raw24 = (uint32_t(b[0]) << 16) | (uint32_t(b[1]) << 8) | b[2];
  return true;
}

bool ADS124S08_Drv::readData(int32_t& value24_signed) {
  uint32_t raw = 0;
  if (!readDataRaw(raw)) return false;
  value24_signed = signExtend24(raw);
  return true;
}

// ===== 默认配置（示例）=====
bool ADS124S08_Drv::defaultConfig() {
  // 0) 软复位 + 小延时
  softReset();
  if (hal_.delay_ms) hal_.delay_ms(5);

  // 1) 设通道: INPMUX = AINP=AIN1, AINN=AIN2
  // 参考TI示例表，INPMUX=0x12 表示 AIN1-AIN2（高4位=P，低4位=N，按手册位图）。:contentReference[oaicite:6]{index=6}
  {
    uint8_t v = 0x12; // AINP=1, AINN=2
    if (!writeRegisters(static_cast<uint8_t>(Reg::INPMUX), &v, 1)) return false;
  }

  // 2) PGA/增益/延迟: PGA=2、保留默认输入延迟；示例：0x0B 是 "PGA使能, Gain=8" 的范例，
  // 你需求是 Gain=2 -> 根据手册位图把 G[2:0] 设为对应值（例如 001=2x）。这里用占位 0x01 仅示意。
  {
    // TODO: 按手册位图设置：PGA_EN=1, GAIN=2x, 可选 DELAY[2:0]
    uint8_t v = 0x09; // 
    if (!writeRegisters(static_cast<uint8_t>(Reg::PGA_GAIN), &v, 1)) return false;
  }

  // 3) 数据率/滤波/模式: 连续转换 + 低延迟滤波 + 20 SPS
  // DR[3:0]=0100 -> 20SPS；FILTER=0 选择低延迟滤波；MODE=连续。:contentReference[oaicite:7]{index=7}
  {
    // 示例：0x14 在TI示例表里代表 "LL, 20SPS, 连续"（注意你的器件位图可能稍异，按手册位图写）
    uint8_t v = 0x14;
    if (!writeRegisters(static_cast<uint8_t>(Reg::DATARATE), &v, 1)) return false;
  }

  // 4) 参考设置：选择外部参考 REFP0/REFN0；同时**内部参考常开**以供IDAC。并使能参考缓冲。
  {
    // 参考TI示例：REF寄存器编码(示例对 REFP1/REFN1)。我们改成 REFP0/REFN0 的选择位。
    // 还需置 "内参Always On" 位（即使外参工作也开内参，满足IDAC要求）。
    uint8_t v = 0x02; 
    // TODO: 按手册位图置：REFSEL=REFP0/REFN0；REFCON=AlwaysOn；REF缓冲使能(正/负)。
    // 例如：v = REFSEL_REFP0 | REFCON_AON | REFP_BUF_EN | REFN_BUF_EN;
    if (!writeRegisters(static_cast<uint8_t>(Reg::REF), &v, 1)) return false;
  }

  // 5) IDAC 设置：IMAG=500uA，路由到 AIN0（IDAC1→AIN0，IDAC2 关）。:contentReference[oaicite:9]{index=9}
  {
    // IDACMAG: IMAG[3:0] = 0101 -> 500uA
    uint8_t imag = 0x05;
    if (!writeRegisters(static_cast<uint8_t>(Reg::IDACMAG), &imag, 1)) return false;

    // IDACMUX: I1MUX 选 AIN0，I2MUX 选 NC（或 AINCOM 关）
    // 手册给出I1MUX/I2MUX编码表，例如 AIN0=0000，NC=1101..1111（取其一）。:contentReference[oaicite:10]{index=10}
    uint8_t imux = 0xF0; 
    // TODO: imux = (I1MUX_AIN0 << I1POS) | (I2MUX_NC << I2POS);
    if (!writeRegisters(static_cast<uint8_t>(Reg::IDACMUX), &imux, 1)) return false;
  }

  // 6) SYS：关 SENDSTAT/CRC，便于简化读数（后续你要链路校验再开）  
  {
    uint8_t sys = 0x10; // [ ... |CRC|STAT ] 两位清0
    if (!writeRegisters(static_cast<uint8_t>(Reg::SYS), &sys, 1)) return false;
  }

  // 8) 写后读回校验（已为您实现打印功能）
Serial.println("--- Verifying Register Settings ---"); // 打印一个标题，方便查看
struct { Reg r; const char* name; } toCheck[] = {
  {Reg::INPMUX,   "INPMUX"},
  {Reg::PGA_GAIN, "PGA"},
  {Reg::DATARATE, "DATARATE"},
  {Reg::REF,      "REF"},
  {Reg::IDACMAG,  "IDACMAG"},
  {Reg::IDACMUX,  "IDACMUX"},
  {Reg::SYS,      "SYS"},
};

bool all_ok = true; // 添加一个标志来跟踪校验结果

for (auto& x : toCheck) {
  uint8_t v = 0;
  if (!readRegisters(static_cast<uint8_t>(x.r), &v, 1)) {
    Serial.printf("Failed to read register: %s\n", x.name);
    all_ok = false;
    continue; // 读取失败，继续检查下一个
  }
  
  // 使用 Serial.printf 打印读回的值
  Serial.printf("  [CHECK] %-8s = 0x%02X\n", x.name, v);
  
  // 您还可以在这里加入判断，如果读回的值和您写入的值不符，就报错
  // 例如，检查 DATARATE 寄存器：
  // if (x.r == Reg::DATARATE && v != 0x14) {
  //   Serial.println("  [ERROR] DATARATE verification failed!");
  //   all_ok = false;
  // }
}

if (!all_ok) {
    Serial.println("--- Verification FAILED! ---");
    return false; // 如果有任何一个寄存器校验失败，则配置失败
}

Serial.println("--- Verification PASSED ---");
  // 9) 启动连续转换
  return start();
}

bool ADS124S08_Drv::readPT1000Temperature(float r_ref, float& temperature)
{
  // 1. 等待数据就绪 (DRDY)
  // 20SPS 模式下，数据周期是 50ms，我们设置 60ms 超时防止死锁
  int32_t raw_signed = 0;
  if(!waitDRDY(60))
  {
    return false;
  }
  // 2. 读取 24位原始数据
  if(!readData(raw_signed))
  {
    return false;
  }
  // 3. 转换为电阻值 (比率测量法)
  const int gain = 2;
  const int FS = 8388608.0; // 2^23
  // 根据公式反推电阻值
  // R_RTD = R_REF * (Output Code) / (Gain * 2^23)
  //       = (R_REF / Gain) * (Output Code / 2^23)
  //       = (V_REF / IDAC / Gain) * (Output Code / 2^23)
  // 注意: voltage = V_REF * (Output Code / (Gain * 2^23))
  // 所以: R_RTD = voltage * R_REF / V_REF = voltage / IDAC
  //计算当前测到的电压，占参考电压的百分比。
  double ratio = (double)raw_signed/FS;
  double r_rtd = ratio * r_ref/(double)gain;
  // --- 简单的异常过滤 ---
  // PT1000 在 -50度是 803欧，在 200度是 1758欧。
  // 如果算出来是 0 或者无穷大，说明断路或短路。
  if(r_rtd <100.0 || r_rtd>5000)
  {
    return false;
  }

  



}

