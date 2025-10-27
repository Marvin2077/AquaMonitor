#include "ad5941_drv.h"
#include <Arduino.h> // For Serial printouts if enabled

// 构造函数：保存 HAL 回调
AD5941_Drv::AD5941_Drv(const Hal& hal) : hal_(hal) {}

// 初始化驱动和芯片
bool AD5941_Drv::begin() {
    // 检查 HAL 指针是否有效 (可选但推荐)
    if (!hal_.cs_assert || !hal_.cs_release || !hal_.spi_txrx || !hal_.delay_us || !hal_.delay_ms) {
        if (Serial) Serial.println("AD5941 Begin Error: HAL functions missing");
        return false; // HAL 不完整
    }
    // 执行强制初始化序列
    if (!initializeChipMandatorySequence()) {
        if (Serial) Serial.println("AD5941 Begin Error: Mandatory init sequence failed");
        return false;
    }
    return true;
}

// 执行软件复位
bool AD5941_Drv::softwareReset() {
    if (Serial) Serial.println("Performing AD5941 software reset...");
    // 软件复位需要密钥 (Datasheet P127)
    if (!writeRegister16(Reg::RSTCONKEY, 0x12EA)) {
        if (Serial) Serial.println("Software Reset Error: Failed to write reset key");
        return false; // 写入密钥
    }
    if (!writeRegister16(Reg::SWRSTCON, 0xA158)) {
        if (Serial) Serial.println("Software Reset Error: Failed to trigger reset");
        return false; // 触发复位
    }
    hal_.delay_ms(5); // 复位后需要短暂延时等待芯片稳定 (Increased delay slightly for safety)
    // 必须重新执行初始化序列
    if (!initializeChipMandatorySequence()) {
        if (Serial) Serial.println("Software Reset Error: Re-initialization failed");
        return false;
    }
    if (Serial) Serial.println("AD5941 software reset complete.");
    return true;
}

// 读取芯片 ID
bool AD5941_Drv::readChipID(uint16_t& adi_id, uint16_t& chip_id) {
    if (!readRegister16(Reg::ADIID, adi_id)) return false; //
    if (!readRegister16(Reg::CHIPID, chip_id)) return false; //
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

    // --- 事务 1: 设置地址 ---
    hal_.cs_assert();
    uint8_t addr_cmd[3] = {
        (uint8_t)SpiCmd::SET_ADDR, // 命令字节
        (uint8_t)(address >> 8),   // 地址高字节
        (uint8_t)(address & 0xFF)  // 地址低字节
    };

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

    if (!hal_.spi_txrx(read_cmd, nullptr, 1)) { // 发送读命令
        if(Serial) Serial.println("  TX2 Cmd FAILED!");
        hal_.cs_release();
        return false;
    }

    // 发送 1 个虚拟字节以启动读取，并忽略 MISO 上的返回
    uint8_t initial_dummy_tx = 0xAA; // 虚拟字节内容不重要
    uint8_t initial_dummy_rx = 0;    // 存储 MISO 返回值的变量（将被丢弃）
    if (!hal_.spi_txrx(&initial_dummy_tx, &initial_dummy_rx, 1)) { // 执行 1 字节 SPI 传输
        if(Serial) Serial.println("  TX2 Initial Dummy FAILED!");
        hal_.cs_release();
        return false;
    }

    // 发送 length 个虚拟字节，同时接收 length 个有效数据字节
    uint8_t data_dummy_tx[length]; // 最多 4 字节
    for (size_t i = 0; i < length; ++i) data_dummy_tx[i] = 0xBB; // 用不同的值区分

    // 执行 SPI 传输，发送虚拟字节，并将接收到的数据存入调用者提供的 data 指针
    if (!hal_.spi_txrx(data_dummy_tx, data, length)) {
        if(Serial) Serial.println("  TX2 Data Transfer FAILED!");
        hal_.cs_release();
        return false;
    }

    hal_.cs_release(); // 结束事务
    return true; // 成功完成读取
}

// 底层 SPI 写寄存器实现 (处理两阶段协议)
bool AD5941_Drv::spiWriteReg(uint16_t address, const uint8_t* data, size_t length) {
     if (!data || (length != 2 && length != 4)) {
         if(Serial) Serial.println("spiWriteReg Error: Invalid length (must be 2 or 4)");
         return false; // 只支持写 16 位或 32 位
     }
     // 检查 HAL 指针
    if (!hal_.cs_assert || !hal_.cs_release || !hal_.spi_txrx || !hal_.delay_us) {
         if(Serial) Serial.println("spiWriteReg Error: HAL functions missing");
         return false;
    }

    // 事务 1: 设置地址
    hal_.cs_assert();
    uint8_t addr_cmd[3] = { (uint8_t)SpiCmd::SET_ADDR, (uint8_t)(address >> 8), (uint8_t)(address & 0xFF) };
    if (!hal_.spi_txrx(addr_cmd, nullptr, 3)) {
        if(Serial) Serial.printf("spiWriteReg Error: TX1 failed for addr 0x%04X\n", address);
        hal_.cs_release();
        return false;
    }
    hal_.cs_release();

    hal_.delay_us(5); // 确保满足 CS 高电平时间 t10 (>= 80ns)

    // 事务 2: 写入数据
    hal_.cs_assert();
    uint8_t write_cmd[1] = { (uint8_t)SpiCmd::WRITE_REG }; //
    if (!hal_.spi_txrx(write_cmd, nullptr, 1)) { // 发送写命令
        if(Serial) Serial.printf("spiWriteReg Error: TX2 cmd failed for addr 0x%04X\n", address);
        hal_.cs_release();
        return false;
    }
    // 发送数据
    if (!hal_.spi_txrx(data, nullptr, length)) {
        if(Serial) Serial.printf("spiWriteReg Error: TX2 data failed for addr 0x%04X\n", address);
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
// 根据 Datasheet Page 29, Table 14
bool AD5941_Drv::initializeChipMandatorySequence() {
    if (Serial) Serial.println("Executing AD5941 mandatory initialization sequence...");
    // 统一使用 32 位写入，高位补零
    if (!writeRegister(0x0908, 0x000002C9)) return false; //
    if (!writeRegister(0x0C08, 0x0000206C)) return false; //
    if (!writeRegister(0x21F0, 0x00000010)) return false; //
    if (!writeRegister(0x0410, 0x000002C9)) return false; //
    if (!writeRegister(0x0A28, 0x00000009)) return false; //
    if (!writeRegister(0x238C, 0x00000104)) return false; //

    // 写入电源模式密钥
    if (!writeRegister(0x0A04, 0x00004859)) return false; // PWRKEY Part 1
    if (!writeRegister(0x0A04, 0x0000F27B)) return false; // PWRKEY Part 2
    // 写入电源模式配置 (Table 14: 0x8009 -> RAMRETEN=1, Active Mode=1)
    if (!writeRegister(0x0A00, 0x00008009)) return false; // PWRMOD

    // 写入电源带宽配置 (Table 14 要求写 0x0000 -> Low power mode, Auto BW)
    if (!writeRegister(0x22F0, 0x00000000)) return false; // PMBW

    hal_.delay_ms(5); // 初始化后稍作延时，确保稳定

    // 读取 ID 验证 SPI 通信
    uint16_t adi_id = 0, chip_id = 0;
    if (!readChipID(adi_id, chip_id)) {
        if (Serial) Serial.println("Init Error: Failed to read chip ID after init sequence!");
        return false;
    }
    if (adi_id != 0x4144) { //
        if (Serial) Serial.printf("Init Error: Wrong ADI ID read: 0x%04X\n", adi_id);
        return false; // ID 不匹配，通信可能存在问题
    }
    if (Serial) Serial.printf("AD5941 Initialized OK. ADI ID: 0x%04X, CHIP ID: 0x%04X\n", adi_id, chip_id);

    return true;
}

// ===== 高层函数占位符 =====

bool AD5941_Drv::startMeasurement() {
    // 这个函数的目标是启动一次配置好的测量（例如，DFT）
    // 通常意味着使能 ADC 转换
    uint32_t afecon = 0;
    if (!readRegister(Reg::AFECON, afecon)) return false;
    // 保险：先确保 ADC 上电 (ADCEN, Bit 7)
    afecon |= (1u << 7);
    // 启动转换 (ADCCONVEN, Bit 8)
    afecon |= (1u << 8);
    return writeRegister(Reg::AFECON, afecon);
}

bool AD5941_Drv::readImpedanceData(int32_t& realPartRaw, int32_t& imaginaryPartRaw) {
    // 读取 DFT 结果寄存器
    uint32_t real_reg, imag_reg;
    if (!readRegister(Reg::DFTREAL, real_reg)) return false; //
    if (!readRegister(Reg::DFTIMAG, imag_reg)) return false; //

    // DFT 结果是 18 位有符号数 (twos complement)，存储在寄存器的低 18 位
    realPartRaw = (real_reg & 0x3FFFF);
    imaginaryPartRaw = (imag_reg & 0x3FFFF);

    // 手动进行符号扩展 (如果最高位 Bit 17 是 1，则前面补 1)
    if (realPartRaw & 0x20000) { // 检查符号位 (Bit 17, 即 0x20000)
        realPartRaw |= 0xFFFC0000; // 将高 14 位设为 1
    }
    if (imaginaryPartRaw & 0x20000) {
        imaginaryPartRaw |= 0xFFFC0000;
    }

    return true;
}

// 配置电导率测量的核心函数
bool AD5941_Drv::configureConductivityMeasurement(float f_hz,
                                                   uint16_t wg_amp_code,   // WGAMPLITUDE 的 11-bit 原码 (0..0x7FF)
                                                   uint32_t dft_n,         // DFT 点数 (e.g., 4 to 16384)
                                                   bool use_high_bw)       // 频率>=80kHz 置 true
{
    if (Serial) Serial.printf("Configuring AD5941 for Cond: f=%.1fHz, amp=0x%X, N=%u, high_bw=%d\n", f_hz, wg_amp_code, dft_n, use_high_bw);

    // -------------------------
    // 0) 频带/功耗：根据频率选择 PMBW
    // -------------------------
    // Low power mode (<80kHz): PMBW.SYSHS=0, SYSBW 根据需要自动或手动设置 (这里设为 0 自动)
    // High power mode (>80kHz): PMBW.SYSHS=1, SYSBW 通常设为 250kHz (11b)
    uint32_t pmbw_val = 0;
    if (use_high_bw) {
        pmbw_val = (1u << 0) | (0x3u << 2); // SYSHS=1, SYSBW=250kHz
    } else {
        pmbw_val = (0u << 0) | (0x0u << 2); // SYSHS=0, SYSBW=Auto (or set manually if needed)
    }
    if (!writeRegister(Reg::PMBW, pmbw_val)) {
        if (Serial) Serial.println("Cond Cfg Error: Failed to write PMBW");
        return false;
    }

    // -------------------------
    // 1) 波形发生器：正弦 + 频率/幅度/相位/直流偏置
    // -------------------------
    // f_out = f_ACLK * FCW / 2^30
    // 默认 f_ACLK = 16MHz (由系统时钟决定，需确认)
    // 注意: high_bw 模式下 ADC 时钟可能是 32MHz, 但系统时钟(驱动波形发生器)通常仍是 16MHz
    // 如果系统时钟被分频，这里的 fACLK 需要相应调整。目前假设系统时钟为 16MHz。
    const double fACLK = 16.0e6;
    uint32_t fcw = (uint32_t)(((double)f_hz * (1ull << 30)) / fACLK); // Use 1ull for 64-bit intermediate
    if (fcw >= (1u << 24)) fcw = (1u << 24) - 1; // FCW is 24 bits

    if (!writeRegister(Reg::WGFCW, fcw)) return false; //
    if (!writeRegister(Reg::WGPHASE, 0u)) return false; // 0 相位
    if (!writeRegister(Reg::WGOFFSET, 0x800u)) return false; // Sinusoid offset = midscale (0 V DC offset relative to common mode)
    if (!writeRegister(Reg::WGAMPLITUDE, (uint32_t)(wg_amp_code & 0x7FF))) return false; //

    // 选择正弦模式（WGCON.TYPESEL=10b）
    // 确保 DAC 校准使能位被设置 (默认值)
    uint32_t wgcon = 0;
    if (!readRegister(Reg::WGCON, wgcon)) return false;
    wgcon &= ~((uint32_t)0x3 << 1); // Clear TYPESEL bits
    wgcon |=  (0x2u << 1);          // TYPESEL=10 : Sinusoid
    wgcon |= (1u << 5);             // Ensure DACGAINCAL=1
    wgcon |= (1u << 4);             // Ensure DACOFFSETCAL=1
    if (!writeRegister(Reg::WGCON, wgcon)) return false;

    // -------------------------
    // 2) HSTIA 正端偏置 & DE0 电阻组合
    // -------------------------
    // HSTIACON：VBIASSEL=00 -> 选择 VBIAS_CAP (1.11V) 作为 HSTIA+ 偏置
    // (之前选 VZERO0 是针对特定应用，对于通用阻抗测量，1.11V 更常用)
    if (!writeRegister(Reg::HSTIACON, 0x00000000)) return false; // VBIASSEL = 00

    // DE0RESCON：为 DE0 通路选择 RLOAD 和 RTIA 组合
    // 示例: 选择 RLOAD=100 Ohm, RTIA = 5 kOhm (Code 0x64 from Table 36/181)
    // 你可以根据实际需要调整这个值
    if (!writeRegister(Reg::DE0RESCON, 0x00000064))  return false; // Example: RLOAD=100, RTIA=5k on DE0 path

    // -------------------------
    // 3) Switch Matrix：把回路真正连起来
    //    - 激励输出 D -> CE0         : DSWFULLCON.D5 = 1
    //    - P 节点 (正反馈) -> RE0   : PSWFULLCON.P5 = 1 (***重要反馈环路***)
    //    - N 节点 (负反馈) -> SE0   : NSWFULLCON.N5 = 1 (经 RLOAD_SE0, 通常为 100 Ohm)
    //    - 电流检测输入 TIA- -> DE0 : TSWFULLCON.T10 = 1 (T9 必须断开)
    //    然后置 SWCON.SWSOURCESEL=1，让 FULLCON 生效
    // -------------------------
    // 先全部清零，再按需置位
    if (!writeRegister(Reg::DSWFULLCON, 0u)) return false;
    if (!writeRegister(Reg::NSWFULLCON, 0u)) return false;
    if (!writeRegister(Reg::PSWFULLCON, 0u)) return false;
    if (!writeRegister(Reg::TSWFULLCON, 0u)) return false;

    uint32_t dsw_val = (1u << 5); // D5=1 -> D to CE0
    uint32_t psw_val = (1u << 4); // P5=1 -> P to RE0 (Datasheet Fig 36 shows P5 connects to RE0, Bit 4 controls P5)
    uint32_t nsw_val = (1u << 5); // N5=1 -> N to SE0 (via Rload_SE0)
    uint32_t tsw_val = (1u << 9); // T10=1 -> TIA- to DE0 (T9 is implicitly 0)

    if (!writeRegister(Reg::DSWFULLCON, dsw_val)) return false;
    if (!writeRegister(Reg::PSWFULLCON, psw_val)) return false;
    if (!writeRegister(Reg::NSWFULLCON, nsw_val)) return false;
    if (!writeRegister(Reg::TSWFULLCON, tsw_val)) return false;

    // SWCON：选择由 xSWFULLCON 驱动 (SWSOURCESEL=1)
    uint32_t swcon = 0;
    if (!readRegister(Reg::SWCON, swcon)) return false;
    swcon |= (1u << 16); // SWSOURCESEL=1
    // 当 SWSOURCESEL=1 时，不需要再操作 SWCON 的 T9CON/T10CON 位
    if (!writeRegister(Reg::SWCON, swcon)) return false;

    // 等待开关稳定 (可选但推荐)
    hal_.delay_us(100);

    // -------------------------
    // 4) ADC/AFE 使能: 确保 ADC, HSDAC, HSTIA, Excitation Buffer, High Speed Reference 都已使能
    // -------------------------
    uint32_t afecon = 0;
    if (!readRegister(Reg::AFECON, afecon)) return false;
    afecon &= ~(1u << 5); // HSREFDIS=0 -> 使能高速参考
    afecon |= (1u << 6);  // DACEN=1    -> 使能 HSDAC
    afecon |= (1u << 7);  // ADCEN=1    -> 使能 ADC
    afecon |= (1u << 9);  // EXBUFEN=1  -> 使能激励缓冲器
    afecon |= (1u << 11); // TIAEN=1    -> 使能高速 TIA
    afecon |= (1u << 14); // WAVEGENEN=1-> 使能波形发生器
    // ADCCONVEN (Bit 8) 在这里不设置，由 startMeasurement() 控制
    afecon &= ~(1u << 8); // 确保 ADCCONVEN = 0
    if (!writeRegister(Reg::AFECON, afecon)) {
        if (Serial) Serial.println("Cond Cfg Error: Failed to write AFECON for enables");
        return false;
    }

    // -------------------------
    // 5) ADC MUX & Gain
    // -------------------------
    // ADCCON: 选择 HSTIA 输出作为正输入, VBIAS_CAP 作为负输入, 设置 PGA Gain
    uint32_t adccon = 0;
    adccon |= (0x9u << 8);  // MUXSELN = VBIAS_CAP (01000b = 8, not 9! Corrected: 0x8u)
    adccon |= (0x1u << 0);  // MUXSELP = HSTIA Positive Output (000001b = 1)
    adccon |= (0x1u << 16); // GNPGA = Gain 1.5 (001b = 1) (或者根据信号幅度选其他增益)
    if (!writeRegister(Reg::ADCCON, adccon)) return false;

    // -------------------------
    // 6) ADC 滤波与 DFT 配置
    // -------------------------
    // ADCFILTERCON: 设置 ADC 速率, SINC3 OSR, 旁路 SINC2 和 Notch Filter
    uint32_t adcf = 0;
    adcf |= (1u << 0); // ADCSAMPLERATE = 1 -> 800 kSPS (for low power mode) (High power mode might need 0 for 1.6MHz)
    if(use_high_bw) {
         adcf &= ~(1u << 0); // ADCSAMPLERATE = 0 -> 1.6 MHz for high power mode
    }
    adcf |= (0x1u << 12); // SINC3OSR = 1 -> OSR 4 (Recommended)
    adcf |= (1u << 6); // SINC3BYP = 1 -> Bypass SINC3 for wider bandwidth at DFT input (重要: 否则高频信号会被 SINC3 衰减)
    // SINC2 和 Notch filter 保持默认旁路或禁用状态 (根据复位值)
    if (!writeRegister(Reg::ADCFILTERCON, adcf)) return false;

    // DFTCON: 设置 DFT 点数 (N), Hanning 窗使能, DFT 输入源
    uint32_t dftnum_code = 9; // Default 2048 points
    if (dft_n == 4) dftnum_code = 0;
    else if (dft_n == 8) dftnum_code = 1;
    else if (dft_n == 16) dftnum_code = 2;
    else if (dft_n == 32) dftnum_code = 3;
    else if (dft_n == 64) dftnum_code = 4;
    else if (dft_n == 128) dftnum_code = 5;
    else if (dft_n == 256) dftnum_code = 6;
    else if (dft_n == 512) dftnum_code = 7;
    else if (dft_n == 1024) dftnum_code = 8;
    else if (dft_n == 2048) dftnum_code = 9;
    else if (dft_n == 4096) dftnum_code = 10;
    else if (dft_n == 8192) dftnum_code = 11;
    else if (dft_n == 16384) dftnum_code = 12;

    uint32_t dftcon = 0;
    dftcon |= (1u << 0); // HANNINGEN = 1 -> Enable Hanning window
    dftcon |= ((dftnum_code & 0xF) << 4); // DFTNUM (Bits 7:4)
    // DFTINSEL (Bits 21:20): Select input after Gain/Offset (with or without SINC3)
    // Since we bypassed SINC3, this selects ADC raw data after gain/offset
    dftcon |= (0x1u << 20); // DFTINSEL = 01b
    if (!writeRegister(Reg::DFTCON, dftcon)) return false;

    // 最后: 确保 DFT 硬件加速器在 AFECON 中使能
    if (!readRegister(Reg::AFECON, afecon)) return false;
    afecon |= (1u << 15); // DFTEN=1
    if (!writeRegister(Reg::AFECON, afecon)) return false;


    if (Serial) Serial.println("AD5941 Configuration for Conductivity Measurement OK.");
    return true;
}


// 检查 DFT 测量是否完成
bool AD5941_Drv::isMeasurementReady() {
  uint32_t flag = 0;
  // DFT 完成标志位在 INTCFLAG0 或 INTCFLAG1 的 Bit 1
  if (!readRegister(Reg::INTCFLAG0, flag)) { // 检查中断标志寄存器 0
      if (!readRegister(Reg::INTCFLAG1, flag)) return false; // 如果 0 读取失败，尝试读取 1
  }

  if (flag & (1u << 1)) { // FLAG1: DFT result ready?
    // 清除标志位 (写 1 清除)
    if (!writeRegister(Reg::INTCCLR, (1u << 1))) {
        // 如果清除失败，可能影响下次检测，但本次结果已准备好
        if (Serial) Serial.println("Warning: Failed to clear DFT ready flag.");
    }
    return true;
  }
  return false; // DFT 结果尚未就绪
}

bool AD5941_Drv::outputSineToCE0(float f_hz, uint16_t amp_code, bool high_bw) {
  // (A) PMBW
  uint32_t pmbw = 0;
  if (high_bw) pmbw = (1u<<0); // SYSHS=1
  if (!writeRegister(Reg::PMBW, pmbw)) return false;

  // (B) WG 参数
  const double fACLK = high_bw ? 16e6 : 16e6; // WG 仍按 16MHz 计算
  uint32_t fcw = (uint32_t)((f_hz * (1ull<<30)) / fACLK);
  if (fcw > 0xFFFFFF) fcw = 0xFFFFFF;
  if (!writeRegister(Reg::WGFCW, fcw)) return false;
  if (!writeRegister(Reg::WGPHASE, 0)) return false;
  if (!writeRegister(Reg::WGOFFSET, 0)) return false;
  if (!writeRegister(Reg::WGAMPLITUDE, amp_code & 0x7FF)) return false;

  // 选择正弦
  uint32_t wgcon = 0;
  if (!readRegister(Reg::WGCON, wgcon)) return false;
  wgcon &= ~0x6;            // 清 TYPESEL
  wgcon |= (2u<<1);         // TYPESEL=10b -> 正弦
  if (!writeRegister(Reg::WGCON, wgcon)) return false;  // WG 设置完成（先别开 WAVEGEN）

  // (C) 只连 D->CE0
  if (!writeRegister(Reg::DSWFULLCON, (1u<<5))) return false; // D5=1
  uint32_t swcon = 0;
  if (!readRegister(Reg::SWCON, swcon)) return false;
  swcon |= (1u<<0); // SWSOURCESEL=1
  if (!writeRegister(Reg::SWCON, swcon)) return false;

  // (D) 打开 DAC 参考、DAC、本激励缓冲，最后开 WAVEGEN
  uint32_t afecon = 0;
  if (!readRegister(Reg::AFECON, afecon)) return false;
  afecon |= (1u<<20); // DACREFEN
  afecon |= (1u<<9);  // EXBUFEN
  afecon |= (1u<<6);  // DACEN
  if (!writeRegister(Reg::AFECON, afecon)) return false;

  afecon |= (1u<<14); // WAVEGENEN
  if (!writeRegister(Reg::AFECON, afecon)) return false;

  return true;
}
