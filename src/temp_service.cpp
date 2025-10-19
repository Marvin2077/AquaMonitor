#include "temp_service.h"

// --- 将 24 位码值转成电阻 ---
double TempService::codeToRx(int32_t code24) const {
  constexpr double FS = 8388608.0; // 2^23
  return (static_cast<double>(code24) / FS) * (cfg_.Rref_ohm / static_cast<double>(cfg_.pga_gain));
}

// --- IEC 60751: PT1000 R->T ---
double TempService::pt1000_R_to_T(double R) {
  const double R0 = 1000.0;
  const double A  = 3.9083e-3;
  const double B  = -5.775e-7;
  const double C  = -4.183e-12;

  // 先猜测温度区间
  if (R >= R0) {
    // T >= 0°C: 解  R = R0*(1 + A*T + B*T^2)
    double x = (R / R0) - 1.0;
    double disc = A*A - 4.0*B*(-x); // A^2 - 4B(1 - R/R0)
    if (disc < 0) disc = 0;
    double T1 = (-A + sqrt(disc)) / (2.0*B);
    return T1;
  } else {
    // T < 0°C: R = R0*(1 + A*T + B*T^2 + C*(T-100)*T^3)
    // 牛顿迭代
    double T = -10.0; // 初值
    for (int i = 0; i < 8; ++i) {
      double f  = R0*(1 + A*T + B*T*T + C*(T-100.0)*T*T*T) - R;
      double df = R0*(A + 2*B*T + C*(4*T*T*T - 300.0*T*T)); // d/dT
      T -= f/df;
    }
    return T;
  }
}

// --- 读一次 ---
bool TempService::readPT1000Once(double& tempC, double* Rrx_out) {
  // 等待数据就绪
  if (!adc_.waitDRDY(200)) return false;

  // 读 24 位码值（双极性补码）
  int32_t code24 = 0;
  if (!adc_.readData(code24)) return false; // 见你的驱动实现 :contentReference[oaicite:0]{index=0}

  // 码值 -> 电阻 -> 温度
  const double Rx = codeToRx(code24);
  if (Rrx_out) *Rrx_out = Rx;

  tempC = pt1000_R_to_T(Rx);
  return true;
}
