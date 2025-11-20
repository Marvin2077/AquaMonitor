#pragma once
#include "Arduino.h"
#include "ads124s08_drv.h"

class TempService
{
public:
  struct Config
  {
    double Rref_ohm = 3300.0;
    int pga_gain = 2;
  };
    // ================= 校准相关结构体 =================
  struct CalibPoint
  {
    double t_true;
    double t_meas;
  };
  struct CalibCoeff
  {
    double a = 0.0;
    double b = 1.0;
    double c = 0.0;    // ax^2 + bx + c

    bool   valid = false;
  };
  explicit TempService(ADS124S08_Drv& adc, const Config& cfg);
  /**
   * @brief 执行一次完整的测量流程
   * 步骤：等待数据 -> 读取 -> 转电阻 -> 硬件检查 -> 转温度 -> 应用校准
   * @param out_temp 输出最终校准后的温度
   * @return true 成功, false 失败(超时/断线/短路)
   */
  bool measure(double& out_temp);
  // ================= 工具/调试 API =================
  // 仅读取原始电阻值 (用于调试或校准采集)
  bool readResistance(double& out_ohm);
  // 将 24 位 ADC 码值转换为电阻 (比率测量法)
  double codeToResistance(int32_t code24) const;
  // PT1000 核心算法：电阻 -> 温度 (Callendar-Van Dusen)
  static double resistanceToTemp(double R);

  // === 分步校准 API ===
  // 第1步：测量并记录某一个校准点 (index: 0, 1, 2)
  // true_temp: 你当前水温的真实值 (比如 0.0, 50.0)
  bool recordCalibPoint(int index, double true_temp);
  bool fitThreePoint(const CalibPoint& p1,const CalibPoint& p2,const CalibPoint& p3,CalibCoeff& coeff);
  // 第2步：当三个点都测完后，调用这个函数计算 a, b, c 并生效
  bool finishCalibration();

  void setCalib(const CalibCoeff& input_data)
  //函数声明写的是 setCalib(const CalibCoeff& ...)，那么你传进来的必须是 CalibCoeff
  {
    calib_ = input_data;
  }
CalibCoeff getCalib() const 
{
  return calib_;
}

double applyCalib(double t_meas) const;

private:
  ADS124S08_Drv& adc_;
  Config cfg_;
  CalibCoeff calib_;
  CalibPoint temp_points_[3];
};

