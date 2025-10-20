#include "cond_service.h"
#include <math.h>

bool CondService::begin(const CondConfig& cfg) {
  cfg_ = cfg;
  // 依赖 ad5941_drv 提供的高层配置函数：
  // 若你还没把 configureConductivityMeasurement()/readImpedanceOnce()
  // 放进驱动，请把我之前给你的实现拷进去。
  return afe_.configureConductivityMeasurement(
            cfg_.freq_hz, cfg_.amp_code, cfg_.dft_n, cfg_.high_bw);
}

bool CondService::calibrate() {
int32_t real_raw = 0, imag_raw = 0;
if (!afe_.readImpedanceData(real_raw, imag_raw)) return false;
double M   = hypot((double)real_raw, (double)imag_raw); // 幅值
double phi = atan2((double)imag_raw, (double)real_raw); // 相位（弧度）
  // 防守：避免零除
  if (M <= 0.0) return false;
  calib_C_ = cfg_.Rcal_ohm * M;
  return true;
}

bool CondService::read_once(CondReading& out) {
int32_t real_raw = 0, imag_raw = 0;
if (!afe_.readImpedanceData(real_raw, imag_raw)) return false;
double M   = hypot((double)real_raw, (double)imag_raw); // 幅值
double phi = atan2((double)imag_raw, (double)real_raw); // 相位（弧度）

  // 没标定也能给出相对值，这里做保护
  double C = (calib_C_ > 0.0) ? calib_C_ : (cfg_.Rcal_ohm * M);

  double Z = C / M;     // |Z| = Rcal * (Mcal / M)；这里把 C 预先存了
  double G = (Z > 0.0) ? (1.0 / Z) : 0.0;
  double sigma = cfg_.cell_K * G;

  out.mag = M;
  out.phase_rad = phi;
  out.Z_abs_ohm = Z;
  out.G_si = G;
  out.sigma_Spcm = sigma;
  return true;
}

bool CondService::reconfig(const CondConfig& cfg) {
  cfg_ = cfg;
  calib_C_ = 0.0; // 重新配置后建议重标定
  return afe_.configureConductivityMeasurement(
            cfg_.freq_hz, cfg_.amp_code, cfg_.dft_n, cfg_.high_bw);
}
