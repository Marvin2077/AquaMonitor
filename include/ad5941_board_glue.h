#ifndef AD5941_BOARD_GLUE_H
#define AD5941_BOARD_GLUE_H

#include <Arduino.h>
#include "spi_hal.h"     // 复用之前的 spi_hal
#include "ad5941_drv.h"  // 包含 AD5941 驱动头文件

// AD5941 Glue 对外配置结构体
struct Ad5941GlueConfig {
    const SpiDevice* spi;      // 指向 SpiDevice 配置 (包含 CS 引脚、速率、模式)
    // uint8_t pin_reset;    // RESET 引脚 (如果硬件有连接) - 0xFF 表示未使用
    // uint8_t pin_interrupt; // 用于接收中断的 GPIO 引脚 (如果使用中断模式) - 0xFF 表示未使用
};

// 对外 API 命名空间
namespace Ad5941BoardGlue {

    /**
     * @brief 创建并返回 AD5941 驱动所需的 HAL 结构体
     * @param cfg 包含 SPI 设备和引脚配置的结构体
     * @return 配置好的 AD5941_Drv::Hal 结构体
     */
    AD5941_Drv::Hal make_ad5941_hal(const Ad5941GlueConfig& cfg);

    /**
     * @brief (可选) 执行硬件复位脉冲
     * @param cfg 包含 RESET 引脚信息的配置结构体
     */
    // void ad5941_hw_reset(const Ad5941GlueConfig& cfg);

} // namespace Ad5941BoardGlue

#endif // AD5941_BOARD_GLUE_H