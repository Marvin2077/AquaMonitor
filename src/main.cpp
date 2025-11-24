#include <Arduino.h>
#include <SPI.h>

//所有驱动层头文件
#include "spi_hal.h"        // 底层硬件操作
#include "ads124s08_board_glue.h"     // 胶水层
#include "ads124s08_drv.h"  // 通用驱动层
#include "ad5941_board_glue.h"      // 引入 Glue 层头文件
#include "ad5941PlatformCfg.h"
#include "temp_service.h" 
#include "impedance_service.h" 
#include "2Wire_service.h"
#include "storage_manager.h"
#include "mux_iface.h"
extern "C" {
#include "ad5940.h"
}

/* ====== 共用 SPI 总线引脚 ====== */
static const int PIN_SCLK = 14; // ESP32 SPI 时钟线
static const int PIN_MISO = 12; // ESP32 SPI MISO线
static const int PIN_MOSI = 13; // ESP32 SPI MOSI线
/* ====== AD5941 特定引脚 ====== */
static const int CS_AD5941    = 27; // AD5941 片选引脚
static const int RESET_AD5941 = 26; // AD5941 复位引脚
/* ====== ADS124S08 特定引脚 ====== */
static const int CS_ADS124S08    = 16; // ADS124S08 片选引脚
static const int RESET_ADS124S08 = 15; // ADS124S08 复位引脚
static const int DRDY_ADS124S08 =  4; // ADS124S08 DRDY引脚
/* ====== 设备ID ====== */
int Deviceid = -1;
//解析串口命令函数
void handleSerialCommand();
// === AD5941 全局变量 ===
// 定义用于存放 AD5941 序列器指令的缓冲区
#define APPBUFF_SIZE 512
uint32_t AppBuff[APPBUFF_SIZE];
// 声明阻抗测量的配置结构体
extern AppIMPCfg_Type AppIMPCfg;
// === 单频/扫频标志 ===
// false = 日常单频模式 (1000Hz)
// true  = 正在进行扫频
bool g_isSweepMode = false;
int g_sweepCount = 0;     // 当前已测量的点数
int g_sweepTotalPoints = 0; // 总共要测的点数
// === ADS124S08 全局变量 ===
// 创建一个 ADS124S08 驱动对象（此时还未初始化）
ADS124S08_Drv* ads124s08 = nullptr;
//创建 Temp Service 对象指针
TempService* g_tempSvc = nullptr;
const double MY_RREF = 3300.0;
bool is_calibrating = false;


void setup() {
  //读取Device id

  Serial.begin(115200);
  ensureDeviceID();
  delay(1000);
  // 初始化测量通道MUX的地址引脚
  // 初始化为通道1
  pinMode(CHANNEL_MUX_ADDR1, OUTPUT);
  digitalWrite(CHANNEL_MUX_ADDR1, HIGH);
  pinMode(CHANNEL_MUX_ADDR0, OUTPUT);
  digitalWrite(CHANNEL_MUX_ADDR0, LOW);
  pinMode(ISFET_MUX_ADDR2, OUTPUT);
  digitalWrite(ISFET_MUX_ADDR2, LOW);
  pinMode(ISFET_MUX_ADDR1, OUTPUT);
  digitalWrite(ISFET_MUX_ADDR1, LOW);
  pinMode(ISFET_MUX_ADDR0, OUTPUT);
  digitalWrite(ISFET_MUX_ADDR0, LOW);

  //初始化 SPI 总线
  SpiHAL::beginBus(PIN_SCLK, PIN_MISO, PIN_MOSI);
  Serial.println("SPI bus initialized.");
  // --- 初始化 AD5941 --- //
  Serial.println("======================================");
  Serial.println(" AD5941 Setup Start ");
  Serial.println("======================================");

  // 1) 创建 AD5941 的 SpiDevice 对象
  static SpiDevice ad5941_spidev(CS_AD5941, 8000000 /* 8MHz */, MSBFIRST, SPI_MODE0);
  Serial.println("SpiDevice for AD5941 created.");

  // 2) 配置并初始化 Ad5941 Glue 层
  Ad5941Glue::Config cfg;
  cfg.spi = &ad5941_spidev;
  cfg.pin_reset = RESET_AD5941;
  Ad5941Glue::setup(cfg);
  Serial.println("Ad5941Glue setup complete.");
  // 3) 执行硬件复位
  Ad5941Glue::hardware_reset(1000, 100);
  Serial.println("AD5941 hardware reset performed.");
  if(AD5941PlatformCfg() == AD5940ERR_OK)
  {
    Serial.println("AD5941 Platform Configuration performed.");
  }

  // --- 初始化 ADS124S08 --- //
  Serial.println("======================================");
  Serial.println(" ADS124S08 Setup Start ");
  Serial.println("======================================");
   // 1)  初始化 ADS124S08 SPI 总线
  static SpiDevice ads_spidev(CS_ADS124S08, 4000000, MSBFIRST, SPI_MODE1);
  //  2)  定义 ADS124S08 的 Glue 配置
  AdsGlueConfig ads_cfg = {
    .spi = &ads_spidev,
    .pin_drdy = DRDY_ADS124S08,
    .pin_reset = RESET_ADS124S08
  };
  Serial.println("Making ADS124S08 HAL...");
  ADS124S08_Drv::Hal ads_hal = BoardGlue::make_ads_hal(ads_cfg);
  // 3) 创建ADS124S08驱动实例
  ads124s08 = new ADS124S08_Drv(ads_hal);
  Serial.println("ADS124S08 Driver instance created.");
  Serial.println("Configuring ADS124S08...");
  if (!ads124s08->defaultConfig()) {
    Serial.println("!!! ADS124S08 configuration FAILED! Halting. !!!");
    while (1) { delay(1000); }
  }
    // --- 完成ADS124S08芯片的初始化 --- //

  // --- 初始化 ADS124S08 温度测量服务 --- //
  TempService::Config tsCfg;
  tsCfg.Rref_ohm = MY_RREF; 
  tsCfg.pga_gain = 2;      
  g_tempSvc = new TempService(*ads124s08, tsCfg);
  Serial.println("ADS124S08 configuration successful.");
  // --- 初始化 AD5941 阻抗测量服务 --- //
  Serial.println("Initializing Impedance Service...");
  //初始化应用参数结构体
  AppBIOZCfg_init();
  // 5. 配置2线应用参数 (修改Rcal和AIN1)
  Serial.println("Running RTIA Calibration... This may take a moment.");
  if(AppBIOZInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK) {
    Serial.println("BIOZ_Impedance Service Init OK!");
  }
  else {
        Serial.println("BIOZ_Impedance Service Init FAILED!");
    }
  /*
  // 1. 加载默认配置
  AppIMPCfg_init();
  //AppBIOZCfg_init();
  // 2. 初始化服务 (生成序列并写入 AD5941 SRAM)
    // 这步很重要，它会生成初始化序列和测量序列
    if (AppIMPInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK) {
        Serial.println("Impedance Service Init OK!");
    } else {
        Serial.println("Impedance Service Init FAILED!");
    }
  */
  //完成芯片和服务初始化
  Serial.println("System Initialized");
  // 测试MUX控制
  ChooseSenesingChannel(1);
  ChooseISFETChannel(2);
}


void loop() {
  static uint32_t last_time = 0;
  uint32_t tempCount = APPBUFF_SIZE;
  
  handleSerialCommand();

  // ============================================
  // 1. 触发部分：每 500ms 告诉 AD5941 开始测量
  // ============================================
  if (millis() - last_time > 30000) {
    last_time = millis();
    
    // 如果不是扫频模式，触发单点测量
    if (!g_isSweepMode) {
       AppBIOZCtrl(BIOZCTRL_START, 0);
    }
  }

  // ============================================
  // 2. 读取部分：持续轮询 (Polling)
  // ============================================
  // 不论是否到了 500ms，只要 AD5941 有数据回来，就立刻处理
  // 这样可以保证数据产生后约 20ms 内就被读走，不会堆积
  
  if (!g_isSweepMode) {
      // 检查 AD5941 是否有新数据
      if (AppBIOZISR(AppBuff, &tempCount) == 0) 
      {
        // 只有当真正读到数据时 (tempCount > 0)
        if (tempCount > 0) 
        {
          BIOZShowResult(AppBuff, tempCount);
          double currentTemp = 0.0;
          // 1. 此时再去测温度，保证时间和阻抗数据对齐
          if (g_tempSvc->measure(currentTemp)) {
             Serial.printf(" Water Temp: %3f \n", currentTemp);
          } else {
             Serial.println("Temp Read Failed!");
          }

          // 2. 打印阻抗/电导率结果
          
        }
      }
  } 
  
  // ============================================
  // 3. 扫频模式的特殊处理 (保持原有逻辑)
  // ============================================
  else {
      // 如果是扫频模式，逻辑稍有不同，这里保留你原本的扫频处理
      if(AppBIOZISR(AppBuff, &tempCount) == 0) {
          if(tempCount > 0) {
              BIOZShowResult(AppBuff, tempCount);
              g_sweepCount++;
              if (g_sweepCount < g_sweepTotalPoints) {
                  AppBIOZCtrl(BIOZCTRL_START, 0); 
              } else {
                  Serial.println(">>> Sweep Completed! <<<");
                  g_isSweepMode = false;
                  // 恢复单点参数...
                  AppBIOZCfg.SinFreq = 2000.0; // 设回你的默认频率
                  AppBIOZCfg.SweepCfg.SweepEn = bFALSE;
                  AppBIOZCfg.bParaChanged = bTRUE;
                  AppBIOZInit(AppBuff, APPBUFF_SIZE); 
              }
          }
      }
  }
}


void handleSerialCommand() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); // 去掉回车换行

    // === 指令: "cal 0" -> 校准 0度点 ===
    if (cmd == "cal 0") {
      Serial.println("Measuring Point 1 (0.0 C)... Keep sensor stable.");
      // 假设这杯水是 0.0 度
      if (g_tempSvc->recordCalibPoint(0, 0.0)) {
         Serial.println("Point 1 Saved!");
      } else {
         Serial.println("Point 1 Failed!");
      }
    }
    
    // === 指令: "cal 50" -> 校准 50度点 ===
    else if (cmd == "cal 50") {
      Serial.println("Measuring Point 2 (50.0 C)...");
      if (g_tempSvc->recordCalibPoint(1, 50.0)) {
         Serial.println("Point 2 Saved!");
      }
    }

    // === 指令: "cal 100" -> 校准 100度点 ===
    else if (cmd == "cal 100") {
      Serial.println("Measuring Point 3 (100.0 C)...");
      if (g_tempSvc->recordCalibPoint(2, 100.0)) {
         Serial.println("Point 3 Saved!");
      }
    }

    // === 指令: "save" -> 计算并生效 ===
    else if (cmd == "save") {
      Serial.println("Computing coefficients...");
      if (g_tempSvc->finishCalibration()) {
        auto c = g_tempSvc->getCalib();
        Serial.printf("Calibration DONE! a=%.6f, b=%.6f, c=%.6f\n", c.a, c.b, c.c);
      } else {
        Serial.println("Calibration Calculation Failed (Check points?)");
      }
    }
    
    // === 指令: "reset" -> 清除校准 ===
    else if (cmd == "reset") {
      TempService::CalibCoeff clean; // 默认是 valid=false
      g_tempSvc->setCalib(clean);
      Serial.println("Calibration Cleared.");
    }
    // === 新增: "imp" -> 触发一次阻抗测量 ===
    else if (cmd == "imp") {
       Serial.println("Triggering Impedance Measurement (1000Hz)...");
       // 所以这里调用它会直接让 AD5941 跑一次 SRAM 里的 SEQID_0 序列
       AppIMPCtrl(IMPCTRL_START, 0);
    }
    else if (cmd == "bioz") {
       Serial.println("Triggering BIOZ_Impedance Measurement (1000Hz)...");
       // 所以这里调用它会直接让 AD5941 跑一次 SRAM 里的 SEQID_0 序列
       AppBIOZCtrl(BIOZCTRL_START, 0);
    }    

    else if (cmd == "sweep") {
       if (g_isSweepMode) {
           Serial.println("Already sweeping!");
           return;
       }
       
       Serial.println("[CMD] Starting Frequency Sweep (1kHz -> 100kHz)...");
       // 1. 配置参数
       AppIMPCfg.SweepCfg.SweepEn = bTRUE;
       AppIMPCfg.SweepCfg.SweepLog = bFALSE;
       AppIMPCfg.bParaChanged = bTRUE;   
       // 3. 初始化服务 (这会计算出 SweepNextFreq = 2000.0)
       if (AppIMPInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK) {
           g_isSweepMode = true;
           // === 核心修复：使用独立计数器 ===
           g_sweepCount = 0; 
           g_sweepTotalPoints = AppIMPCfg.SweepCfg.SweepPoints;
           // 4. 触发第 1 个点 (1000Hz)
           AppIMPCtrl(IMPCTRL_START, 0);
       } else {
           Serial.println("Sweep Init Failed!");
       }
    }
        else if (cmd == "biozsweep") {
       if (g_isSweepMode) {
           Serial.println("Already sweeping!");
           return;
       }
       
       Serial.println("[CMD] Starting Frequency Sweep (1kHz -> 100kHz)...");
       // 1. 配置参数
       AppBIOZCfg.SweepCfg.SweepEn = bTRUE;
       AppBIOZCfg.ReDoRtiaCal = bTRUE;
       AppBIOZCfg.bParaChanged = bTRUE;   
       // 3. 初始化服务 (这会计算出 SweepNextFreq = 2000.0)
       if (AppBIOZInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK) {
           g_isSweepMode = true;
           g_sweepCount = 0; 
           g_sweepTotalPoints = AppBIOZCfg.SweepCfg.SweepPoints;
           // 4. 触发第 1 个点 (1000Hz)
           AppBIOZCtrl(BIOZCTRL_START, 0);
       } else {
           Serial.println("Sweep Init Failed!");
       }
    }
  }
}