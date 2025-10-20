#include "ad5941_drv.h"
#include <Arduino.h>
// 构造函数：保存 HAL 回调
AD5941_Drv::AD5941_Drv(const Hal& hal) : hal_(hal) {}

// 初始化驱动和芯片
bool AD5941_Drv::begin() {
    // 检查 HAL 指针是否有效 (可选但推荐)
    if (!hal_.cs_assert || !hal_.cs_release || !hal_.spi_txrx || !hal_.delay_us || !hal_.delay_ms) {
        return false; // HAL 不完整
    }
    // 执行强制初始化序列
    return initializeChipMandatorySequence();
}

// 执行软件复位
bool AD5941_Drv::softwareReset() {
    // 软件复位需要密钥 (Datasheet P127)
    if (!writeRegister16(Reg::RSTCONKEY, 0x12EA)) return false; // 写入密钥
    if (!writeRegister16(Reg::SWRSTCON, 0xA158)) return false; // 触发复位
    hal_.delay_ms(2); // 复位后需要短暂延时等待芯片稳定
    // 可能需要重新执行 begin() 中的初始化序列
    return initializeChipMandatorySequence();
}

// 读取芯片 ID
bool AD5941_Drv::readChipID(uint16_t& adi_id, uint16_t& chip_id) {
    if (!readRegister16(Reg::ADIID, adi_id)) return false;
    if (!readRegister16(Reg::CHIPID, chip_id)) return false;
    return true;
}

/**
 * @brief 底层 SPI 读取寄存器实现
 *
 * 处理 AD5941 的两阶段协议：
 * 1. 发送 SET_ADDR 命令 + 16 位地址
 * 2. 发送 READ_REG 命令，然后发送 1 个虚拟字节以启动数据传输，
 * 最后再发送 length 个虚拟字节，同时接收 length 个有效数据字节。
 * @param address 要读取的 16 位寄存器地址
 * @param data 指向接收数据的缓冲区的指针
 * @param length 要读取的数据长度 (必须是 2 或 4)
 * @return true 读取成功, false 失败
 */
bool AD5941_Drv::spiReadReg(uint16_t address, uint8_t* data, size_t length) {
    // 参数检查
    if (!data || (length != 2 && length != 4)) {
        if(Serial) Serial.println("spiReadReg Error: Invalid length (must be 2 or 4)");
        return false; // 只支持读 16 位或 32 位
    }
    // 检查 HAL 指针
    if (!hal_.cs_assert || !hal_.cs_release || !hal_.spi_txrx || !hal_.delay_us) {
         if(Serial) Serial.println("spiReadReg Error: HAL functions missing");
         return false;
    }

    if(Serial) Serial.printf("spiReadReg: Attempting to read address 0x%04X, length %d\n", address, length);

    // --- 事务 1: 设置地址 ---
    hal_.cs_assert();
    uint8_t addr_cmd[3] = {
        (uint8_t)SpiCmd::SET_ADDR, // 命令字节
        (uint8_t)(address >> 8),   // 地址高字节
        (uint8_t)(address & 0xFF)  // 地址低字节
    };
    if(Serial) Serial.print("  TX1 (SET_ADDR): ");
    for (int i = 0; i < 3; ++i) { if(Serial) Serial.printf("%02X ", addr_cmd[i]); }
    if(Serial) Serial.println();

    if (!hal_.spi_txrx(addr_cmd, nullptr, 3)) { // 发送命令和地址
        if(Serial) Serial.println("  TX1 FAILED!");
        hal_.cs_release();
        return false;
    }
    hal_.cs_release();

    hal_.delay_us(5); // 确保满足 CS 高电平时间 t10 (>= 80ns)

    // --- 事务 2: 读取数据 ---
    hal_.cs_assert();
    uint8_t read_cmd[1] = { (uint8_t)SpiCmd::READ_REG }; // 读命令字节
    if(Serial) Serial.print("  TX2 (READ_REG Cmd): ");
    if(Serial) Serial.printf("%02X \n", read_cmd[0]);

    if (!hal_.spi_txrx(read_cmd, nullptr, 1)) { // 发送读命令
        if(Serial) Serial.println("  TX2 Cmd FAILED!");
        hal_.cs_release();
        return false;
    }

    // *** 关键修复：发送 1 个虚拟字节以启动读取，并忽略 MISO 上的返回 ***
    uint8_t initial_dummy_tx = 0xAA; // 虚拟字节内容不重要
    uint8_t initial_dummy_rx = 0;    // 存储 MISO 返回值的变量（将被丢弃）
    if(Serial) Serial.print("  TX2 (Initial Dummy): ");
    if(Serial) Serial.printf("%02X \n", initial_dummy_tx);
    if (!hal_.spi_txrx(&initial_dummy_tx, &initial_dummy_rx, 1)) { // 执行 1 字节 SPI 传输
        if(Serial) Serial.println("  TX2 Initial Dummy FAILED!");
        hal_.cs_release();
        return false;
    }
    if(Serial) Serial.print("  RX2 (Discarded after dummy): ");
    if(Serial) Serial.printf("%02X \n", initial_dummy_rx); // 打印被丢弃的字节 (预期是之前的 0x80)

    // *** 现在，发送 length 个虚拟字节，同时接收 length 个有效数据字节 ***
    uint8_t data_dummy_tx[length]; // 最多 4 字节
    for (size_t i = 0; i < length; ++i) data_dummy_tx[i] = 0xBB; // 用不同的值区分

    if(Serial) Serial.print("  TX2 (Data Dummies): ");
    for (size_t i = 0; i < length; ++i) { if(Serial) Serial.printf("%02X ", data_dummy_tx[i]); }
    if(Serial) Serial.println();

    // 执行 SPI 传输，发送虚拟字节，并将接收到的数据存入调用者提供的 data 指针
    if (!hal_.spi_txrx(data_dummy_tx, data, length)) {
        if(Serial) Serial.println("  TX2 Data Transfer FAILED!");
        hal_.cs_release();
        return false;
    }

    // 打印实际接收到的有效数据
    if(Serial) Serial.print("  RX2 (Received Data): ");
    for (size_t i = 0; i < length; ++i) { if(Serial) Serial.printf("%02X ", data[i]); }
    if(Serial) Serial.println();

    hal_.cs_release(); // 结束事务
    if(Serial) Serial.println("spiReadReg: Success.");
    return true; // 成功完成读取
}

// 底层 SPI 写寄存器实现 (处理两阶段协议)
bool AD5941_Drv::spiWriteReg(uint16_t address, const uint8_t* data, size_t length) {
     if (!data || (length != 2 && length != 4)) return false; // 只支持写 16 位或 32 位

    // 事务 1: 设置地址
    hal_.cs_assert();
    uint8_t addr_cmd[3] = { (uint8_t)SpiCmd::SET_ADDR, (uint8_t)(address >> 8), (uint8_t)(address & 0xFF) };
    if (!hal_.spi_txrx(addr_cmd, nullptr, 3)) {
        hal_.cs_release();
        return false;
    }
    hal_.cs_release();

    hal_.delay_us(1); // 两个事务之间的短暂延时

    // 事务 2: 写入数据
    hal_.cs_assert();
    uint8_t write_cmd[1] = { (uint8_t)SpiCmd::WRITE_REG };
    if (!hal_.spi_txrx(write_cmd, nullptr, 1)) { // 发送写命令
        hal_.cs_release();
        return false;
    }
    // 发送数据
    if (!hal_.spi_txrx(data, nullptr, length)) {
        hal_.cs_release();
        return false;
    }
    hal_.cs_release();
    return true;
}


// 读取 32 位寄存器
bool AD5941_Drv::readRegister(Reg address, uint32_t& value) {
    return readRegister((uint16_t)address, value);
}
bool AD5941_Drv::readRegister(uint16_t address, uint32_t& value) {
    uint8_t buffer[4];
    if (!spiReadReg(address, buffer, 4)) return false;
    // MSB first 传输，需要正确组合字节
    value = ((uint32_t)buffer[0] << 24) |
            ((uint32_t)buffer[1] << 16) |
            ((uint32_t)buffer[2] << 8)  |
            ((uint32_t)buffer[3]);
    return true;
}

// 写入 32 位寄存器
bool AD5941_Drv::writeRegister(Reg address, uint32_t value) {
    return writeRegister((uint16_t)address, value);
}
bool AD5941_Drv::writeRegister(uint16_t address, uint32_t value) {
    uint8_t buffer[4] = {
        (uint8_t)(value >> 24),
        (uint8_t)(value >> 16),
        (uint8_t)(value >> 8),
        (uint8_t)(value)
    };
    return spiWriteReg(address, buffer, 4);
}

// 读取 16 位寄存器
bool AD5941_Drv::readRegister16(Reg address, uint16_t& value) {
    return readRegister16((uint16_t)address, value);
}
bool AD5941_Drv::readRegister16(uint16_t address, uint16_t& value) {
    uint8_t buffer[2];
    if (!spiReadReg(address, buffer, 2)) return false;
    value = ((uint16_t)buffer[0] << 8) | buffer[1];
    return true;
}

// 写入 16 位寄存器
bool AD5941_Drv::writeRegister16(Reg address, uint16_t value) {
    return writeRegister16((uint16_t)address, value);
}
bool AD5941_Drv::writeRegister16(uint16_t address, uint16_t value) {
    uint8_t buffer[2] = { (uint8_t)(value >> 8), (uint8_t)(value) };
    return spiWriteReg(address, buffer, 2);
}


// 执行强制初始化序列 (私有函数)
bool AD5941_Drv::initializeChipMandatorySequence() {
    // 写入 Datasheet Page 29, Table 14 中的所有值
    // 注意：这里的寄存器地址是 16 位的，但写入的值可能是 16 位或 32 位
    // 我们统一使用 32 位写入，高位补零即可

    // 注释说明每个写入的目的，如果已知的话
    if (!writeRegister(0x0908, 0x02C9)) return false; // 未知目的，来自 Table 14
    if (!writeRegister(0x0C08, 0x206C)) return false; // 未知目的，来自 Table 14
    if (!writeRegister(0x21F0, 0x0010)) return false; // REPEATADCCNV 设置? 来自 Table 14
    if (!writeRegister(0x0410, 0x02C9)) return false; // CLKEN1 设置? 来自 Table 14
    if (!writeRegister(0x0A28, 0x0009)) return false; // EI2CON 设置? 来自 Table 14
    if (!writeRegister(0x238C, 0x0104)) return false; // ADCBUFCON 设置? 来自 Table 14

    // 写入电源模式密钥
    if (!writeRegister(0x0A04, 0x4859)) return false; // PWRKEY Part 1
    if (!writeRegister(0x0A04, 0xF27B)) return false; // PWRKEY Part 2
    // 写入电源模式配置 (Table 14 示例值 0x8009, 但根据 P129, 0x0001 是 Active Mode)
    // 0x8009 = RAMRETEN=1, ???, PWRMOD=Active. 使用 0x8001 更安全? 先用 Table 14 的
    if (!writeRegister(0x0A00, 0x8009)) return false; // PWRMOD 设置

    // 写入电源带宽配置 (Table 14 要求写 0x0000)
    if (!writeRegister(0x22F0, 0x0000)) return false; // PMBW 清零 (Low power mode, Auto BW)

    hal_.delay_ms(2); // 初始化后稍作延时

    // 读取 ID 验证 SPI 通信
    uint16_t adi_id = 0, chip_id = 0;
    if (!readChipID(adi_id, chip_id)) {
        // Serial.println("Failed to read chip ID after init!"); // 如果有 Serial
        return false;
    }
    if (adi_id != 0x4144) {
        // Serial.printf("Wrong ADI ID: 0x%04X\n", adi_id); // 如果有 Serial
        return false; // ID 不匹配，通信可能存在问题
    }
    // Serial.printf("AD5941 Initialized. ADI ID: 0x%04X, CHIP ID: 0x%04X\n", adi_id, chip_id); // 如果有 Serial

    return true;
}

// ===== 高层函数占位符 =====

bool AD5941_Drv::startMeasurement() {
    uint32_t afecon = 0;
    if (!readRegister(Reg::AFECON, afecon)) return false;
    // 保险：先确保 ADC 上电
    afecon |= AfeConBits::ADCEN;
    // 启动转换
    afecon |= AfeConBits::ADCCONVEN;
    return writeRegister(Reg::AFECON, afecon);
}

bool AD5941_Drv::readImpedanceData(int32_t& realPartRaw, int32_t& imaginaryPartRaw) {
    // TODO: 读取 DFT 结果寄存器
    uint32_t real_reg, imag_reg;
    if (!readRegister(Reg::DFTREAL, real_reg)) return false;
    if (!readRegister(Reg::DFTIMAG, imag_reg)) return false;

    // DFT 结果是 18 位有符号数，存储在低 18 位
    realPartRaw = (real_reg & 0x3FFFF);
    imaginaryPartRaw = (imag_reg & 0x3FFFF);

    // 进行符号扩展 (如果第 18 位是 1，则前面补 1)
    if (realPartRaw & 0x20000) { // 检查符号位 (Bit 17)
        realPartRaw |= 0xFFFC0000; // 将高位设为 1
    }
    if (imaginaryPartRaw & 0x20000) {
        imaginaryPartRaw |= 0xFFFC0000;
    }

    return true;
}

// ad5941_drv.cpp

bool AD5941_Drv::configureConductivityMeasurement(float f_hz,
                                                   uint16_t wg_amp_code,   // WGAMPLITUDE 的 11-bit 原码
                                                   uint32_t dft_n,         // 1024/2048/4096/8192 等
                                                   bool use_high_bw)       // 频率>80kHz 设 true
{
  // 0) 频带/功耗：<80kHz 低功率；≥80kHz 置高功率（PMBW.SYSHS=1），并选合适 SYSBW 截止
  //    这里：低功率 0x0000；高功率举例 0x000D（SYSHS=1，SYSBW=50k/100k/250k自动随 WG）:contentReference[oaicite:6]{index=6} :contentReference[oaicite:7]{index=7}
  if (!writeRegister(Reg::PMBW, use_high_bw ? 0x000D : 0x0000)) return false;

  // 1) 波形发生器参数：先写 WGFCW/WGPHASE/WGOFFSET/WGAMPLITUDE，再开 TYPESEL 与 AFECON.WAVEGENEN
  //    f = 16MHz * FCW / 2^30；WGAMPLITUDE 为 11-bit 幅度码（受 HSDAC 增益/衰减影响）:contentReference[oaicite:8]{index=8} :contentReference[oaicite:9]{index=9}
  const double fACLK = 16e6;
  uint32_t fcw = (uint32_t)((f_hz * (1u<<30)) / fACLK);
  if (fcw > 0xFFFFFF) fcw = 0xFFFFFF;
  if (!writeRegister(Reg::WGFCW, fcw)) return false;
  if (!writeRegister(Reg::WGPHASE, 0u)) return false;
  if (!writeRegister(Reg::WGOFFSET, 0u)) return false;
  if (!writeRegister(Reg::WGAMPLITUDE, (uint32_t)(wg_amp_code & 0x7FF))) return false;

  // 2) 选择正弦模式（WGCON.TYPESEL=10b）
  uint32_t wgcon = 0;
  if (!readRegister(Reg::WGCON, wgcon)) return false;
  wgcon &= ~((uint32_t)0x3 << 1);
  wgcon |= (0x2u << 1); // TYPESEL=10: Sinusoid
  if (!writeRegister(Reg::WGCON, wgcon)) return false;  // :contentReference[oaicite:10]{index=10}

  // 3) 连接开关矩阵：DE0→HSTIA-（T10=1）；确保 T9=0；写完 xSWFULLCON 后需置 SWCON.SWSOURCESEL=1 生效。:contentReference[oaicite:11]{index=11}
  uint32_t tsw = 0;
  tsw |= (1u<<9);  // T10 closed
  tsw |= (0u<<8);  // T9 open
  if (!writeRegister(Reg::TSWFULLCON, tsw)) return false;
  uint32_t swcon = 0;
  if (!readRegister(Reg::SWCON, swcon)) return false;
  swcon |= (1u<<0); // SWSOURCESEL=1: 使用 xSWFULLCON 的设置
  if (!writeRegister(Reg::SWCON, swcon)) return false;

  // 4) 使能 HSDAC（AFECON.DACEN=1），但先别开 WAVEGENEN，最后统一开；ADC 先上电（ADCEN=1）。:contentReference[oaicite:12]{index=12}
  uint32_t afecon = 0;
  if (!readRegister(Reg::AFECON, afecon)) return false;
  afecon |= (1u<<6); // DACEN
  afecon |= (1u<<7); // ADCEN
  if (!writeRegister(Reg::AFECON, afecon)) return false;

  // 5) ADC 滤波 + DFT：DFT 输入选 SINC3 输出；根据 f_hz 设置 OSR 和 DFT 长度 N，建议加 Hanning 窗。:contentReference[oaicite:13]{index=13}
  //    下面给一组保守设置：SINC3 OSR=4、SINC2 OSR=22（举例），请按你的采样率需要细调。
  //    这里只示范把 DFTNUM/HANNING/输入源配置进去；位定义随你头文件映射。
  uint32_t adcf = 0;
  if (!readRegister(Reg::ADCFILTERCON, adcf)) return false;
  // 清/设 OSR 位（这里不展开位域，保持你工程中的位定义）
  // adcf = (adcf & ~MASK) | NEWVAL;
  if (!writeRegister(Reg::ADCFILTERCON, adcf)) return false;

  uint32_t dftcon = 0;
  // 例：DFT 开启、输入=SINC3、HANNING=1、N= dft_n（映射到寄存器的编码由你在头文件里定义）
  // dftcon = (DFTEN=1) | (DFTINSEL=SINC3) | (HANNING=1) | (DFTNUM编码)
  if (!writeRegister(Reg::DFTCON, dftcon)) return false;

  // 6) 最后开波形发生器输出（AFECON.WAVEGENEN=1），并启动 ADC 转换（ADCCONVEN=1）。:contentReference[oaicite:14]{index=14} :contentReference[oaicite:15]{index=15}
  if (!readRegister(Reg::AFECON, afecon)) return false;
  afecon |= (1u<<0);  // WAVEGENEN=1
  afecon |= (1u<<8);  // ADCCONVEN=1
  if (!writeRegister(Reg::AFECON, afecon)) return false;

  return true;
}

bool AD5941_Drv::isMeasurementReady() {
  uint32_t flag = 0;
  if (!readRegister(Reg::INTCFLAG0, flag)) return false;
  if (flag & (1u<<1)) { // FLAG1: DFT result ready
    (void)writeRegister(Reg::INTCCLR, (1u<<1)); // 写 1 清除
    return true;
  }
  return false;
}
