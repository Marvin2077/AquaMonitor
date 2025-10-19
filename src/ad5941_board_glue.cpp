#include "ad5941_board_glue.h"

// 保存 AD5941 的板级状态，让回调能访问
namespace { // 匿名命名空间
    struct Ad5941State {
        const SpiDevice* dev = nullptr;
        // uint8_t pin_reset = 0xFF; // 如果需要 reset
        // uint8_t pin_interrupt = 0xFF; // 如果需要 interrupt
    } g_ad5941;

    // --- 回调函数实现 ---
    // 这些函数会被驱动通过 HAL 结构体调用

    // 拉低片选
    void cs_assert_5941() {
        if (g_ad5941.dev) g_ad5941.dev->cs_low();
    }

    // 拉高片选
    void cs_release_5941() {
        if (g_ad5941.dev) g_ad5941.dev->cs_high();
    }

    // SPI 传输 (直接调用 spi_hal 的 transfer)
    bool spi_txrx_5941(const uint8_t* tx, uint8_t* rx, size_t n) {
        if (!g_ad5941.dev) return false;
        g_ad5941.dev->beginTxn(); // 应用 SPI 设置
        g_ad5941.dev->transfer(tx, rx, n);
        g_ad5941.dev->endTxn();   // 释放 SPI 总线
        return true;
    }

    // 微秒延时
    void delay_us_5941(uint32_t us) {
        SpiHAL::delay_us(us); // 使用 spi_hal 中的延时
    }

    // 毫秒延时
    void delay_ms_5941(uint32_t ms) {
        SpiHAL::delay_ms(ms); // 使用 spi_hal 中的延时
    }

} // namespace

// --- 对外 API 实现 ---
namespace Ad5941BoardGlue {

    AD5941_Drv::Hal make_ad5941_hal(const Ad5941GlueConfig& cfg) {
        // 1. 保存板级状态
        g_ad5941.dev = cfg.spi;
        // g_ad5941.pin_reset = cfg.pin_reset; // 如果有 reset
        // g_ad5941.pin_interrupt = cfg.pin_interrupt; // 如果有 interrupt

        // 2. 初始化相关 GPIO
        if (g_ad5941.dev) {
            g_ad5941.dev->begin(); // 初始化 CS 引脚
        }
        // if (g_ad5941.pin_reset != 0xFF) { ... } // 如果有 reset 引脚，初始化
        // if (g_ad5941.pin_interrupt != 0xFF) { pinMode(g_ad5941.pin_interrupt, INPUT_PULLUP); ... } // 如果有中断引脚，初始化

        // 3. 返回填充好的 HAL 结构体
        AD5941_Drv::Hal hal = {
            .cs_assert = cs_assert_5941,
            .cs_release = cs_release_5941,
            .spi_txrx = spi_txrx_5941,
            .delay_us = delay_us_5941,
            .delay_ms = delay_ms_5941
            // 注意没有 read_drdy
        };
        return hal;
    }

    // 可选的硬件复位函数
    // void ad5941_hw_reset(const Ad5941GlueConfig& cfg) {
    //     if (cfg.pin_reset != 0xFF) {
    //         digitalWrite(cfg.pin_reset, LOW);
    //         SpiHAL::delay_us(10); // 根据手册确定复位脉冲宽度 (P127, >1us)
    //         digitalWrite(cfg.pin_reset, HIGH);
    //         SpiHAL::delay_ms(2); // 复位后的稳定时间
    //     }
    // }

} // namespace Ad5941BoardGlue