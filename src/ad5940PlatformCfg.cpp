#include "ad5940PlatformCfg.h"

int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;     // 时钟配置结构体
  FIFOCfg_Type fifo_cfg;   // FIFO 配置结构体
  AGPIOCfg_Type gpio_cfg;  // 模拟 GPIO 配置结构体

  /* 使用硬件复位引脚 (如果有连接) 复位 AD5940 */
  AD5940_HWReset();
  /* 初始化 AD5940 芯片，唤醒芯片并检查 SPI 通信是否正常 */
  AD5940_Initialize();

  /* 平台配置 */

  /* 步骤1. 配置时钟 */
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;        // ADC 时钟分频：1分频
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;    // ADC 时钟源：高频振荡器 (HFOSC, 16MHz或32MHz)
  clk_cfg.SysClkDiv = SYSCLKDIV_1;        // 系统时钟分频：1分频
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;    // 系统时钟源：高频振荡器 (HFOSC)
  clk_cfg.HfOSC32MHzMode = bFALSE;        // HFOSC 32MHz 模式：关闭 (即使用 16MHz)
  clk_cfg.HFOSCEn = bTRUE;                // 使能高频振荡器
  clk_cfg.HFXTALEn = bFALSE;              // 禁能高频晶体 (HFXTAL)
  clk_cfg.LFOSCEn = bTRUE;                // 使能低频振荡器 (LFOSC, 32kHz)，用于看门狗和睡眠定时器
  AD5940_CLKCfg(&clk_cfg);                // 应用时钟配置

  /* 步骤2. 配置 FIFO 和序列器 */
  fifo_cfg.FIFOEn = bFALSE;               // 暂时禁能 FIFO (在配置期间)
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;      // FIFO 模式：设置为标准 FIFO 模式 (不是流模式)
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;       // FIFO 大小：设置为 4KB (总共6KB SRAM, 剩下 2KB 给序列器)
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;         // FIFO 数据源：设置为 DFT (数字傅里叶变换) 结果
  fifo_cfg.FIFOThresh = 4;                // FIFO 阈值：设置为 4。
                                          // (DFT结果包含实部和虚部，每次测量(RCAL和Rz)产生2个复数，即4个uint32_t)
  AD5940_FIFOCfg(&fifo_cfg);              // 应用 FIFO 配置 (此时 FIFO 仍是禁能的)
  fifo_cfg.FIFOEn = bTRUE;                // 使能 FIFO
  AD5940_FIFOCfg(&fifo_cfg);              // 再次应用配置，正式使能 FIFO

  /* 步骤3. 中断控制器 */
  // 配置中断控制器1 (INTC1)，使其能产生所有中断源的标志位，但不一定触发中断引脚
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT); // 清除所有中断标志
  // 配置中断控制器0 (INTC0)，使其在 "数据FIFO达到阈值" (DATAFIFOTHRESH) 时产生中断
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT); // 再次清除所有中断标志

  /* 步骤4: 重新配置 GPIO */
  // (AGPIO 是 AD5940 上的可配置引脚)
  gpio_cfg.FuncSet = GP0_INT|GP1_SLEEP|GP2_SYNC; // GP0: 作为中断输出; GP1: 作为睡眠模式指示; GP2: 作为同步信号
  gpio_cfg.InputEnSet = 0;                       // 禁能所有引脚的输入功能
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin2; // 使能 Pin0, Pin1, Pin2 的输出功能
  gpio_cfg.OutVal = 0;                           // 输出值 (锁存器) 设为 0
  gpio_cfg.PullEnSet = 0;                        // 禁能所有引脚的上拉/下拉电阻
  AD5940_AGPIOCfg(&gpio_cfg);                    // 应用 GPIO 配置

  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* 写入解锁密钥，允许 AFE (模拟前端) 进入睡眠模式 */
  return 0;
}
