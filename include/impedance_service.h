#ifndef _IMPEDANCE_SERVICE_H_
#define _IMPEDANCE_SERVICE_H_
#include "stdio.h"
#include "string.h"
#include "math.h"
extern "C"
{
    #include "ad5940.h"
}

#define MAXSWEEP_POINTS 100

typedef struct 
{
  /*所有服务的通用配置*/
  BoolFlag bParaChanged; /*指示是否需要再次生成序列。ImpedanceServiceInit会自动清除此标志*/
  uint32_t SeqStartAddr; /*AD5940 SRAM中的初始化序列起始序列*/
  uint32_t MaxSeqLen;    /*限制最大序列长度*/
  uint32_t SeqStartADDrcal; /* SRAM测量序列起始地址 */
  uint32_t MaxSeqLenCal; 
  /*服务相关参数*/
  float ImpODR;
  int32_t NumOfData; /*该位设置为-1则测量不会停止，如果设置为其他数值则在拿到数据后停止测量*/
  float WuptClkFreq; /*唤醒定时器的时钟频率，一般为32kHz。不需要唤醒定时器*/
  float SysClkFreq; /*当前系统时钟频率*/
  float AdcClkFreq; /*当前ADC时钟频率*/
  float RcalVal; /*校准电阻RCal阻值，单位为Ohm*/
  /*开关矩阵配置*/
  uint32_t DswitchSel;
  uint32_t PswitchSel;
  uint32_t NswitchSel;
  uint32_t TswitchSel;

  uint32_t PwrMod; /*控制芯片功率模式 LP/HP */
  uint32_t HstiaRtiaSel; /*高速TIA的RTIA选择，选择内部电阻 RTIA_INT_200, RTIA_INT_1K, RTIA_INT_5K, RTIA_INT_10K, RTIA_INT_20K, RTIA_INT_40K, RTIA_INT_80K, RTIA_INT_160K*/
  uint32_t ExcitBufGain; /*激励放大器的增益 EXCTBUFGAIN_2, EXCTBUFGAIN_0P25*/
  uint32_t HsDacGain; /*高速DAC的增益 HSDACGAIN_1, HSDACGAIN_0P2*/
  uint32_t HsDacUpdateRate; /*高速DAC的更新率*/
  float DacVoltPP; /*DAC输出峰峰值，单位是mV，最大值为800mVpp*/
  float BiasVolt; /*激励信号是DC叠加AC，这个参数决定DC偏置，单位是mV。0.0mV代表无DC偏置*/
  float SinFreq; /*激励信号频率*/
  uint32_t DftNum; /*DFT数*/
  uint32_t DftSrc; /*DFT数据来源*/
  BoolFlag HanwinEn; /*汉宁窗使能*/
  uint32_t AdcPgaGain; /*ADC的PGA增益 GNPGA_1, GNPGA_1_5, GNPGA_2, GNPGA_4, GNPGA_9*/
  /*-------注意，必须输入ADC信号电压的范围是在±1.5V以内-------*/
  uint8_t ADCSinc3Osr;
  uint8_t ADCSinc2Osr;
  uint8_t ADCAvgNum;
  /*实例化扫频结构体*/
  SoftSweepCfg_Type SweepCfg;
  uint32_t FifoThresh;  /*FIFO threshold. Should be N*4*/
  /*Private variables for internal usage*/
  float SweepCurrFreq;
  float SweepNextFreq;
  float FreqofData;
  BoolFlag IMPInited;
  SEQInfo_Type InitSeqInfo;
  SEQInfo_Type MeasureSeqInfo;
  BoolFlag StopRequired;
  uint32_t FifoDataCount;

}AppIMPCfg_Type;

// 定义一个简单的配置结构体，方便从 main 传参
typedef struct {
    float startFreq;
    float stopFreq;
    int   points;
    float voltage_mVpp; // 激励电压
} ImpSweepConfig_t;

#define IMPCTRL_START 0
#define IMPCTRL_STOPNOW 1
#define IMPCTRL_STOPSYNC 2
#define IMPCTRL_GETFREQ 3
#define IMPCTRL_SHUTDOWN 4

int32_t AppInit(uint32_t *pBuffer, uint32_t BufferSize);
int32_t AppIMPGetCfg(void *pCfg);
int32_t AppIMPISR(void *pBuff, uint32_t *pCount);
int32_t AppIMPCtrl(uint32_t Command, void *pPara);
AD5940Err AppIMPCfg_init();
extern AppIMPCfg_Type AppIMPCfg;
static AD5940Err AppIMPSeqMeasureGen(void);
int32_t AppIMPInit(uint32_t *pBuffer, uint32_t BufferSize);
int32_t ImpedanceShowResult(uint32_t *pData, uint32_t DataCount);



#endif