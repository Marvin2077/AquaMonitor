#pragma once
#include <Arduino.h>
#include "ads124s08_drv.h"

class TempService {
public:
  struct Config {
    double Rref_ohm = 1000.0;   // 参考电阻
    int    pga_gain = 2;        // 你在 ADS124S08 上设定的 PGA 增益
  };

  explicit TempService(ADS124S08_Drv& adc, const Config& cfg)
  : adc_(adc), cfg_(cfg) {}

  // 读一次温度（摄氏度）
  bool readPT1000Once(double& tempC, double* Rrx_out = nullptr);

  // 将 24 位码值 -> Rx(Ω)
  double codeToRx(int32_t code24) const;

  // 将 Rx(Ω) -> T(°C)
  static double pt1000_R_to_T(double R);

  struct CalibPoint {
    double t_true;   // 已知“真实”温度（例如冰水=0.0，恒温槽=25.0、50.0）
    double t_meas;   // 通过 readPT1000Once() 多次均值测到的“原始”温度
  };
  struct CalibCoeff {
    // 二次多项式系数: T_true = a*T_meas^2 + b*T_meas + c
    double a = 0.0, b = 1.0, c = 0.0;
    bool   valid = false;
  };

  // 采样若干次取均值，得到某一校准点的 t_meas
  bool collectCalibPoint(double t_true, int samples, CalibPoint& out);

  // 用三个点拟合二次多项式（稳妥消除微小非线性）
  static bool fitThreePoint(const CalibPoint& p1,
                            const CalibPoint& p2,
                            const CalibPoint& p3,
                            CalibCoeff& coeff);

  // 整体流程：依次采集三点、拟合系数并保存
  bool runThreePointCalibration(double t1_true, double t2_true, double t3_true,
                                int samples_each, CalibCoeff& out);

  // 应用校准：把 readPT1000Once() 得到的 t_meas 转为校准后的温度
  double applyCalib(double t_meas) const;

  // 设置/查询系数（便于以后从NVS/EEPROM加载保存）
  void setCalib(const CalibCoeff& c) { calib_ = c; }
  CalibCoeff getCalib() const { return calib_; }

private:
  ADS124S08_Drv& adc_;
  Config cfg_;
  CalibCoeff calib_;
};

