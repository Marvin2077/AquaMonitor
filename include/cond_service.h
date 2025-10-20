#pragma once
#include <stdint.h>
#include <Arduino.h>
#include "ad5941_drv.h"

struct CondConfig {
  float     freq_hz     = 10000.0f; // 激励频率
  uint16_t  amp_code    = 300;      // WGAMPLITUDE 11-bit 原码
  uint32_t  dft_n       = 4096;     // DFT长度（和你的驱动保持一致的编码/实际N）
  bool      high_bw     = false;    // ≥80kHz 设 true
  double    cell_K      = 1.0;      // 电极池常数 (1/cm)，做电阻测试时可置 1
  double    Rcal_ohm    = 1000.0;   // 校准电阻
  uint32_t  timeout_ms  = 1000;     // 单次等待超时
};

struct CondReading {
  // 原始DFT
  double mag;        // DFT 幅值
  double phase_rad;  // DFT 相位
  // 换算
  double Z_abs_ohm;  // 阻抗幅值
  double G_si;       // 电导 (S)
  double sigma_Spcm; // 电导率 (S/cm) = K * G
};

class CondService {
public:
  explicit CondService(AD5941_Drv& afe) : afe_(afe) {}

  // 初始化测量路径（WG/HSTIA/TSW/DFT等）
  bool begin(const CondConfig& cfg);

  // 用已知电阻Rcal标定：保存 C = Rcal * Mcal
  bool calibrate();

  // 读一次（阻抗/电导/电导率）
  bool read_once(CondReading& out);

  // 动态调参（例如切频）
  bool reconfig(const CondConfig& cfg);

  // 获取/设置配置
  inline const CondConfig& cfg() const { return cfg_; }
  inline double calibC() const { return calib_C_; }

private:
  AD5941_Drv& afe_;
  CondConfig  cfg_;
  double      calib_C_ = 0.0;  // = Rcal * Mcal
};
