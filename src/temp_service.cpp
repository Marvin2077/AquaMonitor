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

bool TempService::collectCalibPoint(double t_true, int samples, CalibPoint& out) {
  if (samples <= 0) return false;
  double sum = 0.0;
  int okcnt = 0;
  for (int i = 0; i < samples; ++i) {
    double t_meas = 0.0;
    if (readPT1000Once(t_meas, nullptr)) {
      sum += t_meas;
      ++okcnt;
    } else {
      // 轻微等待，避免忙等
      delay(10);
    }
  }
  if (okcnt == 0) return false;
  out.t_true = t_true;
  out.t_meas = sum / okcnt;
  return true;
}

static inline bool solve3x3(double A[3][3], double y[3], double x[3]) {
  // 简单的 3x3 高斯消元（无主元选取，足够用于温度校准这类良态问题）
  for (int i = 0; i < 3; ++i) {
    double pivot = A[i][i];
    if (fabs(pivot) < 1e-12) return false;
    double inv = 1.0 / pivot;
    for (int j = i; j < 3; ++j) A[i][j] *= inv;
    y[i] *= inv;
    for (int r = 0; r < 3; ++r) if (r != i) {
      double f = A[r][i];
      for (int c = i; c < 3; ++c) A[r][c] -= f * A[i][c];
      y[r] -= f * y[i];
    }
  }
  x[0] = y[0]; x[1] = y[1]; x[2] = y[2];
  return true;
}

bool TempService::fitThreePoint(const CalibPoint& p1,
                                const CalibPoint& p2,
                                const CalibPoint& p3,
                                CalibCoeff& coeff) {
  // 构建方程: [t1^2 t1 1][a b c]^T = T1_true
  double A[3][3] = {
    { p1.t_meas*p1.t_meas, p1.t_meas, 1.0 },
    { p2.t_meas*p2.t_meas, p2.t_meas, 1.0 },
    { p3.t_meas*p3.t_meas, p3.t_meas, 1.0 }
  };
  double y[3] = { p1.t_true, p2.t_true, p3.t_true };
  double x[3] = { 0,0,0 };
  if (!solve3x3(A, y, x)) return false;
  coeff.a = x[0]; coeff.b = x[1]; coeff.c = x[2];
  coeff.valid = true;
  return true;
}

bool TempService::runThreePointCalibration(double t1_true, double t2_true, double t3_true,
                                           int samples_each, CalibCoeff& out) {
  CalibPoint p1{}, p2{}, p3{};
  if (!collectCalibPoint(t1_true, samples_each, p1)) return false;
  if (!collectCalibPoint(t2_true, samples_each, p2)) return false;
  if (!collectCalibPoint(t3_true, samples_each, p3)) return false;
  if (!fitThreePoint(p1, p2, p3, out)) return false;
  calib_ = out;
  return true;
}

double TempService::applyCalib(double t_meas) const {
  if (!calib_.valid) return t_meas; // 未校准则直通
  return calib_.a*t_meas*t_meas + calib_.b*t_meas + calib_.c;
}
