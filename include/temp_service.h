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

private:
  ADS124S08_Drv& adc_;
  Config cfg_;
};
