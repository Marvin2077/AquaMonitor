#include "cond_service.h"

static inline int32_t sign_extend18(uint32_t v18) {
  v18 &= 0x3FFFF;                 // 18-bit
  if (v18 & 0x20000) v18 |= 0xFFFC0000; // 符号扩展
  return (int32_t)v18;
}

bool CondService::begin(const CondConfig& cfg) {
  cfg_ = cfg;
  calib_C_ = 0.0;

  // 依赖 ad5941_drv 提供的配置函数：
  //   - 设定 PMBW 高/低带宽
  //   - 配置 WG 正弦（FCW/AMP/PHASE/OFFSET）
  //   - 关T9开T10将 DE0 接到 HSTIA-，并选择合适的 HSTIA 阻值（DE0RESCON/HSRTIACON）
  //   - 使能 HSDAC/ADC，配置 DFT 输入源与长度，最后开 WAVEGEN 与 ADCCONVEN
  // 这些寄存器/位定义见数据手册：PMBW、AFECON、DE0RESCON/HSRTIACON、DFT 等。
  return afe_.configureConductivityMeasurement(cfg_.freq_hz, cfg_.amp_code, cfg_.dft_n, cfg_.high_bw);
}

bool CondService::wait_ready_(uint32_t timeout_ms) {
  const uint32_t t0 = millis();
  while ((millis() - t0) < timeout_ms) {
    if (afe_.isMeasurementReady()) return true;
    delay(1);
  }
  return false;
}

bool CondService::calibrate() {
  if (!wait_ready_(cfg_.timeout_ms)) return false;

  int32_t real_raw = 0, imag_raw = 0;
  if (!afe_.readImpedanceData(real_raw, imag_raw)) return false;

  // 标定：以已知电阻 Rcal 的 DFT 幅值 Mcal，计算 calib_C = Rcal * Mcal
  const double Mcal = hypot((double)real_raw, (double)imag_raw);
  if (Mcal <= 0.0) return false;
  calib_C_ = cfg_.Rcal_ohm * Mcal;
  return true;
}

bool CondService::read_once(CondReading& out) {
  if (!wait_ready_(cfg_.timeout_ms)) return false;

  uint32_t rreg = 0, ireg = 0;
  if (!afe_.readRegister(AD5941_Drv::Reg::DFTREAL, rreg)) return false;
  if (!afe_.readRegister(AD5941_Drv::Reg::DFTIMAG, ireg)) return false;

  out.real_raw  = sign_extend18(rreg);
  out.imag_raw  = sign_extend18(ireg);
  out.mag       = hypot((double)out.real_raw, (double)out.imag_raw);
  out.phase_rad = atan2((double)out.imag_raw, (double)out.real_raw);

  // 用 calib_C_ 将 |DFT| 映射到 |Z|：  Z = calib_C / |DFT|
  // 注：DFT 原码与真实阻抗之间的比例由激励幅值、增益、路径等共同决定，标定可吸收比例因子。
  if (calib_C_ > 0.0 && out.mag > 0.0) {
    out.Z_abs_ohm = calib_C_ / out.mag;
  } else {
    out.Z_abs_ohm = NAN;
  }

  // 电导 G=1/Z；电导率 σ = G * K（K 的单位为 1/cm，这里输出 S/cm）
  if (out.Z_abs_ohm > 0.0) {
    out.G_si       = 1.0 / out.Z_abs_ohm;
    out.sigma_Spcm = out.G_si * cfg_.cell_K;
  } else {
    out.G_si = out.sigma_Spcm = NAN;
  }
  return true;
}

bool CondService::reconfig(const CondConfig& cfg) {
  cfg_ = cfg;
  calib_C_ = 0.0; // 重新配置后需要重新标定
  return afe_.configureConductivityMeasurement(cfg_.freq_hz, cfg_.amp_code, cfg_.dft_n, cfg_.high_bw);
}
