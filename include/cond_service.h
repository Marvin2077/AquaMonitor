#pragma once
#include <stdint.h>
#include <Arduino.h>
#include <math.h>
#include "ad5941_drv.h"

// —— 用户可调的测量参数 ——
// 说明：cell_K 为电极池常数(1/cm)。若先用已知电阻做标定/测试，可将 cell_K=1.
struct CondConfig {
  float     freq_hz     = 10000.0f; // 激励频率
  uint16_t  amp_code    = 300;      // WGAMPLITUDE 的 11-bit 原码（0~2047）
  uint32_t  dft_n       = 4096;     // DFT长度（1024/2048/4096/8192 等）
  bool      high_bw     = false;    // 频率 ≥80kHz 建议 true（走高速环路）:contentReference[oaicite:0]{index=0}
  double    cell_K      = 1.0;      // 电极池常数 (1/cm)
  double    Rcal_ohm    = 1000.0;   // 用于标定的已知电阻
  uint32_t  timeout_ms  = 1000;     // 单次等待超时
};

// 一次读数的结果
struct CondReading {
  int32_t real_raw = 0;     // DFT 实部原码（符号扩展到 32 位）
  int32_t imag_raw = 0;     // DFT 虚部原码
  double  mag      = 0.0;   // |DFT|
  double  phase_rad= 0.0;   // 相位（弧度）
  double  Z_abs_ohm= 0.0;   // 阻抗模值(Ω)
  double  G_si     = 0.0;   // 电导(S)
  double  sigma_Spcm=0.0;   // 电导率(S/cm)
};

class CondService {
public:
  explicit CondService(AD5941_Drv& afe) : afe_(afe) {}

  // 初始化测量通道（WG/HSTIA/开关矩阵/DFT 等）
  bool begin(const CondConfig& cfg);

  // 用已知电阻 Rcal_ohm 做一次标定：保存 calib_C = Rcal * Mcal
  bool calibrate();

  // 读一次（阻抗/电导/电导率）
  bool read_once(CondReading& out);

  // 动态重配（如切换频率/幅度/DFT 长度等）——会清空上次的标定系数，需重标定
  bool reconfig(const CondConfig& cfg);

  // 获取/设置配置
  inline const CondConfig& cfg() const { return cfg_; }
  inline double calibC() const { return calib_C_; }

private:
  bool wait_ready_(uint32_t timeout_ms);

private:
  AD5941_Drv& afe_;
  CondConfig  cfg_;
  double      calib_C_ = 0.0;  // = Rcal * Mcal，Mcal 为标定时 |DFT|
};
