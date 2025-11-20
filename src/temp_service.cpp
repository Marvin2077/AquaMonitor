#include "temp_service.h"
#include <math.h>
#include "Arduino.h"
//构造函数的成员初始化列表
TempService :: TempService(ADS124S08_Drv& adc, const Config& cfg) : adc_(adc),cfg_(cfg){}
//TempService (类的名字)::(的) TempService(函数的名字)
// ':' 它的意思是：“在进入大括号 {} 执行任何代码之前，先立刻把下面这些变量填好。”
// 传入 adc 是类ADS124S08_Drv的实例的引用 传入cfg 是 Config 这个结构体的一个实例的引用。
// 构造函数在使用关键字 new 或者声明变量时自动触发的 如 g_tempSvc = new TempService(*ads124s08, tsCfg);
// --- 工具函数 ---
// --- 数学工具：解 3x3 线性方程组 (高斯消元) ---
// 这是一个文件内部的静态辅助函数，不需要在头文件声明
static inline bool solve3x3(double A[3][3], double y[3], double x[3]) {
  for (int i = 0; i < 3; ++i) {
    double pivot = A[i][i];
    if (fabs(pivot) < 1e-12) return false; // 主元过小，无解
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

// --- 拟合函数：根据三个点计算 a, b, c ---
bool TempService::fitThreePoint(const CalibPoint& p1,
                                const CalibPoint& p2,
                                const CalibPoint& p3,
                                CalibCoeff& coeff) {
  // 我们要解方程组: T_true = a*T_meas^2 + b*T_meas + c
  // 构建矩阵方程: [t^2  t  1] * [a b c]^T = T_true
  
  double A[3][3] = {
    { p1.t_meas*p1.t_meas, p1.t_meas, 1.0 },
    { p2.t_meas*p2.t_meas, p2.t_meas, 1.0 },
    { p3.t_meas*p3.t_meas, p3.t_meas, 1.0 }
  };
  
  double Y[3] = { p1.t_true, p2.t_true, p3.t_true }; // 目标值 (真实温度)
  double X[3] = { 0, 0, 0 };                         // 未知数 (a, b, c)

  // 调用上面的数学工具解方程
  if (!solve3x3(A, Y, X)) return false;

  // 保存结果
  coeff.a = X[0]; 
  coeff.b = X[1]; 
  coeff.c = X[2];
  coeff.valid = true;
  return true;
}
// --- 电阻值转化内位温度 ---
// --- 1. 核心转换：码值 -> 电阻 (比率测量法) ---
double TempService::codeToResistance(int32_t code24) const{
  //“常成员函数” (Const Member Function)。
  constexpr double FS = 8388608.0;
  //“编译期常量” (Constant Expression)。它表示在编译代码的时候，这个数就已经算好了，完全确定了。
  // 这里的逻辑：Ratio = Code / FS
  // R_RTD = Ratio * R_REF / Gain
  double ratio = static_cast<double>(code24) /FS;
  Serial.printf("ratio: %.2f %\n", ratio*1000);
  //“静态类型转换” static_cast转换操作的名字 <double>目标类型 (code24)被转换的对象
  //code24是 int32_t (整数),需要改成浮点数避免小数部分被切掉
  double res = ratio * cfg_.Rref_ohm / static_cast<double>(cfg_.pga_gain);
  return res;
}
// --- 2. 核心算法：电阻 -> 温度 (Callendar-Van Dusen) ---
double TempService::resistanceToTemp(double R)
{
  constexpr double R0 = 1000.0;
  constexpr double A  = 3.9083e-3;
  constexpr double B  = -5.775e-7;
  constexpr double C  = -4.183e-12; // 仅用于 T < 0

  if(R >= R0)
  {
    // 公式: R = R0 * (1 + A*T + B*T^2)
    // 变形为一元二次方程: B*T^2 + A*T + (1 - R/R0) = 0
    // 使用求根公式 T = (-A + sqrt(A^2 - 4*B*Z)) / 2B
    double Z = 1.0 - (R / R0);
    double discriminant = (A * A) - (4 * B * Z);
    
    if (discriminant < 0) return -999.0; // 数学错误保护
    double temp= (-A + sqrt(discriminant)) / (2.0 * B);
    return temp;
  }
  else {
    // T < 0°C (冰水混合物以下)，使用更复杂的公式 + 牛顿迭代法逼近
    double T = -10.0; // 初始猜测值
    for (int i = 0; i < 10; ++i) {
      double R_guess = R0 * (1 + A*T + B*T*T + C*(T-100.0)*T*T*T);
      double deriv   = R0 * (A + 2*B*T + C*(4*T*T*T - 300.0*T*T)); // 导数
      double error   = R_guess - R;
      if (fabs(error) < 0.001) break; // 精度足够则退出
      T -= error / deriv;
      }
    return T;
    }
}
// --- 3. 获取原始电阻 (带硬件保护) ---
bool TempService::readResistance(double& out_ohm)
{
  // A. 等待数据就绪 (超时 100ms)
  //开始调用ads124s08类中的方法
  if(!adc_.waitDRDY(100))
  {
    return false;
  }
  // B. 从驱动读取 24位 原始数据
  int32_t raw_code = 0;
  if(!adc_.readData(raw_code))
  {
    return false;
  }
  // C. 转换为电阻
  double r_val = codeToResistance(raw_code);
  // D. 硬件故障检测 (断线或短路)
  // PT1000: 0°C=1000Ω, 100°C=1385Ω
  // 如果读到 <100Ω (短路) 或 >5000Ω (断路/未接)，则认为无效
  if(r_val <100.0 || r_val > 5000.0)
  {
    return false;
  }
  out_ohm = r_val;
  return true;
}
// --- 4. 综合测量函数 (外部调用的主入口) ---
bool TempService::measure(double& out_temp)
{
  double r_meas = 0.0;
  // 1.读取电阻
  if(!readResistance(r_meas))
  {
    return false;
  }
  // 2. 转换温度
  double t_raw = resistanceToTemp(r_meas);
  // 3. 应用校准系数
  out_temp = applyCalib(t_raw);
  return true;

}
// --- 5. 校准应用 ---
double TempService::applyCalib(double t_meas) const
{
  if(!calib_.valid) 
  {
    return t_meas;// 如果没校准，直接返回原始值
  }
  else
  {
    double t_calib = (calib_.a * t_meas * t_meas) + (calib_.b * t_meas) + calib_.c;
    return t_calib;
  }
}
// 记录单点校准数据
bool TempService::recordCalibPoint(int index, double true_temp) {
  if (index < 0 || index > 2) return false;

  // 1. 连续读 10 次取平均，减少噪声干扰
  double sum_meas = 0.0;
  int count = 0;
  for (int i = 0; i < 10; i++) {
    double r_meas = 0.0;
    // 这里的 false 表示不使用校准，我们要存的是“原始值”！
    if (readResistance(r_meas)) { 
       double t_raw = resistanceToTemp(r_meas); // 只转温度，不应用 applyCalib
       sum_meas += t_raw;
       count++;
    }
    delay(50); // 稍微间隔一下
  }

  if (count == 0) return false; // 读失败了

  // 2. 存入暂存区
  temp_points_[index].t_true = true_temp;      // 真实值 (比如 0.0)
  temp_points_[index].t_meas = sum_meas / count; // 测量值 (比如 0.5)
  
  return true;
}

// 完成校准计算
bool TempService::finishCalibration() {
  CalibCoeff new_coeffs;
  
  // 利用你现有的 fitThreePoint 函数进行数学拟合
  if (fitThreePoint(temp_points_[0], temp_points_[1], temp_points_[2], new_coeffs)) {
    // 拟合成功，更新系数
    calib_ = new_coeffs;
    calib_.valid = true; 
    return true;
  }
  return false;
}