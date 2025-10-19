#ifndef AD5941_DRV_H
#define AD5941_DRV_H

#include <stdint.h> // 用于 uint8_t, uint16_t, uint32_t 等
#include <stddef.h> // 用于 size_t

/**
 * @brief AD5941 High Precision Analog Front End Driver
 *
 * 驱动 AD5941 芯片的基础类，提供寄存器访问和基本控制功能。
 * 通过 HAL (Hardware Abstraction Layer) 结构体与底层硬件交互。
 */
class AD5941_Drv {
public:
    // ===== SPI 命令字节 (Datasheet Page 104, Table 126) =====
    enum class SpiCmd : uint8_t {
        SET_ADDR   = 0x20, // 设置后续读/写的寄存器地址
        WRITE_REG  = 0x2D, // 写入之前设置地址的寄存器
        READ_REG   = 0x6D, // 读取之前设置地址的寄存器
        READ_FIFO  = 0x5F  // 读取数据 FIFO
    };

    // ===== 部分重要寄存器地址 (根据需要从 Datasheet P25 开始添加) =====
    enum class Reg : uint16_t {
        // 配置寄存器 (P25, Table 8)
        AFECON          = 0x2000, // AFE 配置寄存器
        PMBW            = 0x22F0, // 电源模式配置寄存器

        // 识别寄存器 (P28, Table 11)
        ADIID           = 0x0400, // Analog Devices ID (应为 0x4144)
        CHIPID          = 0x0404, // 芯片 ID 和版本

        // 复位相关 (P127, Table 161)
        SWRSTCON        = 0x0424, // 软件复位控制
        RSTCONKEY       = 0x0A5C, // SWRSTCON 的密钥
        RSTSTA          = 0x0A40, // 复位状态

        // 时钟相关 (P132, Table 171)
        CLKCON0         = 0x0408, // 时钟分频配置
        CLKSEL          = 0x0414, // 时钟源选择
        OSCKEY          = 0x0A0C, // OSCCON 的密钥
        OSCCON          = 0x0A10, // 振荡器控制
        HSOSCCON        = 0x20BC, // 高速振荡器配置

        // 电源模式 (P129, Table 165)
        PWRMOD          = 0x0A00, // 电源模式配置
        PWRKEY          = 0x0A04, // PWRMOD 的密钥

        // 高速 DAC (P47, Table 25 & 28)
        HSDACCON        = 0x2010, // HSDAC 配置
        HSDACDAT        = 0x2048, // HSDAC 数据
        // ... HSDAC 校准寄存器 ...

        // 波形发生器 (P99, Table 113)
        WGCON           = 0x2014, // WG 配置
        WGFCW           = 0x2030, // WG 正弦频率控制字
        WGAMPLITUDE     = 0x203C, // WG 正弦幅度
        WGOFFSET        = 0x2038, // WG 正弦偏置
        WGPHASE         = 0x2034, // WG 正弦相位
        // ... WG 梯形波相关寄存器 ...

        // 高速 TIA (P52, Table 37)
        HSRTIACON       = 0x20F0, // HS Rtia 配置
        DEORESCON       = 0x20F8, // DE0 引脚 Rtia/Rload 配置
        HSTIACON        = 0x20FC, // HSTIA 配置 (Vbias源选择)

        // ADC 控制 (P58, Table 42)
        ADCCON          = 0x21A8, // ADC 配置 (MUX, PGA)
        ADCFILTERCON    = 0x2044, // ADC 滤波器配置
        ADCDAT          = 0x2074, // ADC 原始结果

        // DFT (P58, Table 42)
        DFTCON          = 0x20D0, // DFT 配置
        DFTREAL         = 0x2078, // DFT 实部结果
        DFTIMAG         = 0x207C, // DFT 虚部结果

        // 开关矩阵 (P76, Table 82)
        SWCON           = 0x200C, // 开关矩阵配置 (简化模式)
        DSWFULLCON      = 0x2150, // D 开关完整配置
        NSWFULLCON      = 0x2154, // N 开关完整配置
        PSWFULLCON      = 0x2158, // P 开关完整配置
        TSWFULLCON      = 0x215C, // T 开关完整配置
        // ... 开关状态寄存器 ...

        // FIFO (P93, Table 94)
        FIFOCON         = 0x2008, // FIFO 配置
        DATAFIFORD      = 0x206C, // 数据 FIFO 读取
        CMDFIFOWADDR    = 0x21D4, // 命令 FIFO 写入地址
        CMDFIFOWRITE    = 0x2070, // 命令 FIFO 写入数据
        CMDDATACON      = 0x21D8, // 命令/数据 FIFO 控制
        DATAFIFOTHRES   = 0x21E0, // 数据 FIFO 阈值
        FIFOCNTSTA      = 0x2200, // FIFO 计数状态

        // 中断 (P113, Table 139)
        INTCPOL         = 0x3000, // 中断极性
        INTCCLR         = 0x3004, // 中断清除
        INTCSEL0        = 0x3008, // 中断选择 0
        INTCSEL1        = 0x300C, // 中断选择 1
        INTCFLAG0       = 0x3010, // 中断标志 0
        INTCFLAG1       = 0x3014, // 中断标志 1
    };

    // ===== AFECON (0x2000) 位定义 (部分) (Datasheet P25, Table 9) =====
    struct AfeConBits {
        static constexpr uint32_t DACEN       = (1UL << 6);  // 使能高速 DAC
        static constexpr uint32_t ADCEN       = (1UL << 7);  // 使能 ADC 电源
        static constexpr uint32_t ADCCONVEN   = (1UL << 8);  // 启动 ADC 转换
        static constexpr uint32_t EXBUFEN     = (1UL << 9);  // 使能激励缓冲器
        static constexpr uint32_t INAMPEN     = (1UL << 10); // 使能激励端仪表放大器 (通常在 4 线测量中使用)
        static constexpr uint32_t TIAEN       = (1UL << 11); // 使能高速 TIA
        static constexpr uint32_t TEMPSENSEN  = (1UL << 12); // 使能温度传感器
        static constexpr uint32_t TEMPCONVEN  = (1UL << 13); // 启动温度转换
        static constexpr uint32_t WAVEGENEN   = (1UL << 14); // 使能波形发生器
        static constexpr uint32_t DFTEN       = (1UL << 15); // 使能 DFT 硬件加速器
        static constexpr uint32_t SINC2EN     = (1UL << 16); // 使能 Sinc2/50Hz/60Hz 滤波器 (注意: 阻抗测量时通常禁用 DFTEN=1 时 SINC2EN=0)
        static constexpr uint32_t DACREFEN    = (1UL << 20); // 使能高速 DAC 参考
        static constexpr uint32_t DACBUFEN    = (1UL << 21); // 使能低功耗 DAC 的 DC 缓冲 (用于 AC 耦合)
        // ... 根据需要添加其他位 ...
    };


    // ===== HAL 回调函数指针结构体 =====
    // 与 ADS124S08 类似，但不包含 DRDY 读取
    struct Hal {
        void (*cs_assert)();                                    // 回调：拉低 CS
        void (*cs_release)();                                   // 回调：拉高 CS
        bool (*spi_txrx)(const uint8_t* tx, uint8_t* rx, size_t n); // 回调：SPI 传输 n 字节
        void (*delay_us)(uint32_t us);                          // 回调：微秒延时
        void (*delay_ms)(uint32_t ms);                          // 回调：毫秒延时
        // 注意：AD5941 没有专门的 DRDY 引脚，数据就绪状态通常通过轮询状态寄存器或中断获取
    };

    /**
     * @brief 构造函数
     * @param hal 包含底层硬件操作函数指针的 HAL 结构体
     */
    explicit AD5941_Drv(const Hal& hal);

    /**
     * @brief 初始化驱动和芯片
     * 包括执行芯片上电后必需的初始化序列 (Datasheet P29, Table 14)
     * @return true 初始化成功, false 失败
     */
    bool begin();

    /**
     * @brief 执行软件复位 (通过 SWRSTCON 寄存器)
     * @return true 复位成功, false 失败 (例如密钥写入失败)
     */
    bool softwareReset();

    /**
     * @brief 读取芯片 ID 和版本号
     * @param adi_id (输出) ADI 公司 ID (应为 0x4144)
     * @param chip_id (输出) 包含器件 ID 和版本号
     * @return true 读取成功, false 失败
     */
    bool readChipID(uint16_t& adi_id, uint16_t& chip_id);

    // ===== 寄存器底层访问 =====

    /**
     * @brief 读取一个 32 位寄存器
     * @param address 寄存器地址 (使用 Reg enum)
     * @param value (输出) 读取到的 32 位值
     * @return true 读取成功, false SPI 通信失败
     */
    bool readRegister(Reg address, uint32_t& value);
    bool readRegister(uint16_t address, uint32_t& value); // 重载以允许直接使用地址

    /**
     * @brief 写入一个 32 位寄存器
     * @param address 寄存器地址 (使用 Reg enum)
     * @param value 要写入的 32 位值
     * @return true 写入成功, false SPI 通信失败
     */
    bool writeRegister(Reg address, uint32_t value);
    bool writeRegister(uint16_t address, uint32_t value); // 重载

    /**
     * @brief 读取一个 16 位寄存器 (通常用于 ID 等)
     * @param address 寄存器地址
     * @param value (输出) 读取到的 16 位值
     * @return true 读取成功, false 失败
     */
    bool readRegister16(Reg address, uint16_t& value);
    bool readRegister16(uint16_t address, uint16_t& value);

    /**
     * @brief 写入一个 16 位寄存器 (注意: AD5941 大部分寄存器是 32 位的)
     * @param address 寄存器地址
     * @param value 要写入的 16 位值
     * @return true 写入成功, false 失败
     */
    bool writeRegister16(Reg address, uint16_t value);
    bool writeRegister16(uint16_t address, uint16_t value);


    // ===== 高层配置和测量函数 (占位符) =====

    /**
     * @brief 配置芯片用于电导率测量
     * @param excitationFreq 激励信号频率 (Hz)
     * @param excitationAmpPeak 激励信号峰值电压 (V) - 注意实际配置的是 DAC 幅度码值
     * @param rtiaOhms 选择的高速 TIA 增益电阻 (Ohm)
     * @param // ... 可能需要更多参数，例如电极连接配置、PGA增益等
     * @return true 配置成功, false 失败
     */
    bool configureConductivityMeasurement(float excitationFreq, float excitationAmpPeak, uint32_t rtiaOhms /* ... */);

    /**
     * @brief 启动一次测量 (通常是启动 ADC 转换)
     * @return true 启动成功, false 失败
     */
    bool startMeasurement();

    /**
     * @brief 检查测量是否完成 (例如，检查 DFT 完成标志)
     * @return true 测量完成, false 未完成或查询失败
     */
    bool isMeasurementReady();

    /**
     * @brief 读取阻抗测量结果 (如果使用 DFT)
     * @param realPart (输出) 阻抗实部 (需要进行单位转换)
     * @param imaginaryPart (输出) 阻抗虚部 (需要进行单位转换)
     * @return true 读取成功, false 失败或数据未就绪
     */
    bool readImpedanceData(int32_t& realPartRaw, int32_t& imaginaryPartRaw); // 先读取原始值


private:
    /**
     * @brief 执行芯片上电后的强制初始化序列 (Datasheet P29, Table 14)
     * @return true 初始化成功, false 写入失败
     */
    bool initializeChipMandatorySequence();

    /**
     * @brief 底层 SPI 读写实现，处理 AD5941 的两阶段协议
     */
    bool spiReadReg(uint16_t address, uint8_t* data, size_t length);
    bool spiWriteReg(uint16_t address, const uint8_t* data, size_t length);


    Hal hal_; // 保存 HAL 回调函数指针
};

#endif // AD5941_DRV_H