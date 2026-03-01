#include "Conductivity_service.h"
#include "Arduino.h"

AppCondCfg_Type AppCondCfg;

// ================= 电导率三点校准全局变量 =================
CondCalibCoeff g_condCalib;
CondCalibPoint g_condCalPoints[3];

/* Default LPDAC resolution(2.5V internal reference). */
#define DAC12BITVOLT_1LSB   (2200.0f/4095)  //mV
#define DAC6BITVOLT_1LSB    (DAC12BITVOLT_1LSB*64)  //mV

AD5940Err AppCondCfg_init(){

    AppCondCfg.bParaChanged = bFALSE;
    AppCondCfg.SeqStartAddr = 0;
    AppCondCfg.MaxSeqLen = 0;
    AppCondCfg.SeqStartAddrCal = 0;
    AppCondCfg.MaxSeqLenCal = 0;

    AppCondCfg.SysClkFreq = 16000000.0;
    AppCondCfg.WuptClkFreq = 32000.0;
    AppCondCfg.AdcClkFreq = 16000000.0;
    AppCondCfg.CondODR = 20.0; /* 20.0 赫兹 */
    AppCondCfg.NumOfData = -1;
    AppCondCfg.RcalVal = 1000.0; /* 1k 欧姆 */

    AppCondCfg.PwrMod = AFEPWR_HP;
    AppCondCfg.HstiaRtiaSel = HSTIARTIA_1K;
    AppCondCfg.CtiaSel = 4;
    AppCondCfg.ExcitBufGain = EXCITBUFGAIN_2;
    AppCondCfg.HsDacGain = HSDACGAIN_1;
    AppCondCfg.HsDacUpdateRate = 7;
    AppCondCfg.DacVoltPP = 600.0;
    AppCondCfg.BiasVolt = -0.0f;

    AppCondCfg.SinFreq = 10000.0; /* 10000 赫兹 */

    AppCondCfg.ADCPgaGain = ADCPGA_1;
    AppCondCfg.ADCSinc3Osr = ADCSINC3OSR_2;
    AppCondCfg.ADCSinc2Osr = ADCSINC2OSR_22;
    AppCondCfg.ADCAvgNum =ADCAVGNUM_16;

    AppCondCfg.DftNum = DFTNUM_16384;
    AppCondCfg.DftSrc = DFTSRC_SINC3;
    AppCondCfg.HanWinEn = bTRUE;

    AppCondCfg.DswitchSel = SWD_CE0;
    AppCondCfg.PswitchSel = SWP_CE0;
    AppCondCfg.NswitchSel = SWN_AIN0;
    AppCondCfg.TswitchSel = SWT_AIN0;

    AppCondCfg.SweepCfg.SweepEn = bFALSE;
    AppCondCfg.SweepCfg.SweepStart = 1000.0;
    AppCondCfg.SweepCfg.SweepStop = 100000.0;
    AppCondCfg.SweepCfg.SweepPoints = 50;
    AppCondCfg.SweepCfg.SweepLog = bFALSE;
    AppCondCfg.SweepCfg.SweepIndex = 0;

    AppCondCfg.FifoThresh = 4; /* 当 SweepEn = bTRUE 时必须为 4 */
    AppCondCfg.CondInited = bFALSE;
    AppCondCfg.StopRequired = bFALSE;

    return AD5940ERR_OK;
}

/**
该函数供上位控制器调用，用于**获取并修改应用参数**（尤其是用户自定义的参数）。
*/
AD5940Err AppCondGetCfg(void *pCfg)
{
  if(pCfg){
    *(AppCondCfg_Type**)pCfg = &AppCondCfg;
    return AD5940ERR_OK;
  }
  return AD5940ERR_PARA;
}

AD5940Err AppCondCtrl(int32_t BcmCtrl, void *pPara)
{
  switch (BcmCtrl)
  {
  case CondCTRL_START:
    {
      AD5940_SEQMmrTrig(SEQID_0);
      break;
    }
  case CondCTRL_STOPNOW:
    {
      if(AD5940_WakeUp(10) > 10)  /* 通过读寄存器唤醒 AFE，最多读取 10 次 */
        return AD5940ERR_WAKEUP;  /* 唤醒失败 */
      /* 立即停止 Wupt */
      AD5940_WUPTCtrl(bFALSE);
#ifdef ADI_DEBUG
      ADI_Print("Cond Stop Now...\n");
#endif
      break;
    }
  case CondCTRL_STOPSYNC:
    {
#ifdef ADI_DEBUG
      ADI_Print("Cond Stop SYNC...\n");
#endif
      AppCondCfg.StopRequired = bTRUE;
      break;
    }
  case CondCTRL_GETFREQ:
    if(pPara)
    {
      if(AppCondCfg.SweepCfg.SweepEn == bTRUE)
        *(float*)pPara = AppCondCfg.FreqofData;
      else
        *(float*)pPara = AppCondCfg.SinFreq;
    }
    break;
  case CondCTRL_SHUTDOWN:
    {
      AppCondCtrl(CondCTRL_STOPNOW, 0);  /* 如果测量正在运行，则停止它。 */
      /* 关闭那些不受休眠操作自动控制的 LPloop 相关模块 */
      AFERefCfg_Type aferef_cfg;
      LPLoopCfg_Type lp_loop;
      memset(&aferef_cfg, 0, sizeof(aferef_cfg));
      AD5940_REFCfgS(&aferef_cfg);
      memset(&lp_loop, 0, sizeof(lp_loop));
      AD5940_LPLoopCfgS(&lp_loop);
      AD5940_EnterSleepS();  /* 进入休眠模式 */
#ifdef ADI_DEBUG
      ADI_Print("Cond Shut down...\n");
#endif
    }
    break;
  default:
    break;
  }
  return AD5940ERR_OK;
}
float AppCondGetCurrFreq(void)
{
  if(AppCondCfg.SweepCfg.SweepEn == bTRUE)
    return AppCondCfg.FreqofData;
  else
    return AppCondCfg.SinFreq;
}

/* 生成初始化序列 */
static AD5940Err AppCondSeqCfgGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  uint32_t const *pSeqCmd;
  uint32_t SeqLen;
  AFERefCfg_Type aferef_cfg;
  HSLoopCfg_Type HsLoopCfg;
  DSPCfg_Type dsp_cfg;
  float sin_freq;

  /* 在此处启动序列发生器 */
  AD5940_SEQGenCtrl(bTRUE);
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);  /* Init all to disable state */
  aferef_cfg.HpBandgapEn = bTRUE;
  aferef_cfg.Hp1V1BuffEn = bTRUE;
  aferef_cfg.Hp1V8BuffEn = bTRUE;
  aferef_cfg.Disc1V1Cap = bFALSE;
  aferef_cfg.Disc1V8Cap = bFALSE;
  aferef_cfg.Hp1V8ThemBuff = bFALSE;
  aferef_cfg.Hp1V8Ilimit = bFALSE;
  aferef_cfg.Lp1V1BuffEn = bFALSE;
  aferef_cfg.Lp1V8BuffEn = bFALSE;

  /* LP 基准控制 - 关闭它们以节省功耗*/
    /* LP reference control - turn off them to save power*/
  if(AppCondCfg.BiasVolt != 0.0f)    /* With bias voltage */
  {
    aferef_cfg.LpBandgapEn = bTRUE;
    aferef_cfg.LpRefBufEn = bTRUE;
  }
  else
  {
    aferef_cfg.LpBandgapEn = bFALSE;
    aferef_cfg.LpRefBufEn = bFALSE;
  }
  aferef_cfg.LpRefBoostEn = bFALSE;
  AD5940_REFCfgS(&aferef_cfg);	
  HsLoopCfg.HsDacCfg.ExcitBufGain = AppCondCfg.ExcitBufGain;
  HsLoopCfg.HsDacCfg.HsDacGain = AppCondCfg.HsDacGain;
  HsLoopCfg.HsDacCfg.HsDacUpdateRate = AppCondCfg.HsDacUpdateRate;

  HsLoopCfg.HsTiaCfg.DiodeClose = bFALSE;
	if(AppCondCfg.BiasVolt != 0.0f)    /* With bias voltage */
		HsLoopCfg.HsTiaCfg.HstiaBias = HSTIABIAS_VZERO0;
	else
		HsLoopCfg.HsTiaCfg.HstiaBias = HSTIABIAS_1P1;
  HsLoopCfg.HsTiaCfg.HstiaCtia = 31; /* 31pF + 2pF */
  HsLoopCfg.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
  HsLoopCfg.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_OPEN;
  HsLoopCfg.HsTiaCfg.HstiaRtiaSel = AppCondCfg.HstiaRtiaSel;

  HsLoopCfg.SWMatCfg.Dswitch = AppCondCfg.DswitchSel;
  HsLoopCfg.SWMatCfg.Pswitch = AppCondCfg.PswitchSel;
  HsLoopCfg.SWMatCfg.Nswitch = AppCondCfg.NswitchSel;
  HsLoopCfg.SWMatCfg.Tswitch = SWT_TRTIA|AppCondCfg.TswitchSel;

  HsLoopCfg.WgCfg.WgType = WGTYPE_SIN;
  HsLoopCfg.WgCfg.GainCalEn = bTRUE;
  HsLoopCfg.WgCfg.OffsetCalEn = bTRUE;
  if(AppCondCfg.SweepCfg.SweepEn == bTRUE)
  {
    AppCondCfg.FreqofData = AppCondCfg.SweepCfg.SweepStart;
    AppCondCfg.SweepCurrFreq = AppCondCfg.SweepCfg.SweepStart;
    AD5940_SweepNext(&AppCondCfg.SweepCfg, &AppCondCfg.SweepNextFreq);
    sin_freq = AppCondCfg.SweepCurrFreq;
  }
  else
  {
    sin_freq = AppCondCfg.SinFreq;
    AppCondCfg.FreqofData = sin_freq;
  }
  HsLoopCfg.WgCfg.SinCfg.SinFreqWord = AD5940_WGFreqWordCal(sin_freq, AppCondCfg.SysClkFreq);
  HsLoopCfg.WgCfg.SinCfg.SinAmplitudeWord = (uint32_t)(AppCondCfg.DacVoltPP/800.0f*2047 + 0.5f);
  HsLoopCfg.WgCfg.SinCfg.SinOffsetWord = 0;
  HsLoopCfg.WgCfg.SinCfg.SinPhaseWord = 0;
  AD5940_HSLoopCfgS(&HsLoopCfg);
  if(AppCondCfg.BiasVolt != 0.0f)    /* With bias voltage */
  {
    LPDACCfg_Type lpdac_cfg;
    
    lpdac_cfg.LpdacSel = LPDAC0;
    lpdac_cfg.LpDacVbiasMux = LPDACVBIAS_12BIT; /* Use Vbias to tuning BiasVolt. */
    lpdac_cfg.LpDacVzeroMux = LPDACVZERO_6BIT;  /* Vbias-Vzero = BiasVolt */
    lpdac_cfg.DacData6Bit = 0x40>>1;            /* Set Vzero to middle scale. */
    if(AppCondCfg.BiasVolt<-1100.0f) AppCondCfg.BiasVolt = -1100.0f + DAC12BITVOLT_1LSB;
    if(AppCondCfg.BiasVolt> 1100.0f) AppCondCfg.BiasVolt = 1100.0f - DAC12BITVOLT_1LSB;
    lpdac_cfg.DacData12Bit = (uint32_t)((AppCondCfg.BiasVolt + 1100.0f)/DAC12BITVOLT_1LSB);
    lpdac_cfg.DataRst = bFALSE;      /* Do not reset data register */
    lpdac_cfg.LpDacSW = LPDACSW_VBIAS2LPPA|LPDACSW_VBIAS2PIN|LPDACSW_VZERO2LPTIA|LPDACSW_VZERO2PIN|LPDACSW_VZERO2HSTIA;
    lpdac_cfg.LpDacRef = LPDACREF_2P5;
    lpdac_cfg.LpDacSrc = LPDACSRC_MMR;      /* Use MMR data, we use LPDAC to generate bias voltage for LPTIA - the Vzero */
    lpdac_cfg.PowerEn = bTRUE;              /* Power up LPDAC */
    AD5940_LPDACCfgS(&lpdac_cfg);
  }
  dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_HSTIA_N;
  dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_HSTIA_P;
  dsp_cfg.ADCBaseCfg.ADCPga = AppCondCfg.ADCPgaGain;
  
  memset(&dsp_cfg.ADCDigCompCfg, 0, sizeof(dsp_cfg.ADCDigCompCfg));
  
  dsp_cfg.ADCFilterCfg.ADCAvgNum = AppCondCfg.ADCAvgNum;
  dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ;	/* Tell filter block clock rate of ADC*/
  dsp_cfg.ADCFilterCfg.ADCSinc2Osr = AppCondCfg.ADCSinc2Osr;
  dsp_cfg.ADCFilterCfg.ADCSinc3Osr = AppCondCfg.ADCSinc3Osr;
  dsp_cfg.ADCFilterCfg.BpNotch = bTRUE;
  dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE;
  dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;
  dsp_cfg.DftCfg.DftNum = AppCondCfg.DftNum;
  dsp_cfg.DftCfg.DftSrc = AppCondCfg.DftSrc;
  dsp_cfg.DftCfg.HanWinEn = AppCondCfg.HanWinEn;
  
  memset(&dsp_cfg.StatCfg, 0, sizeof(dsp_cfg.StatCfg));
  AD5940_DSPCfgS(&dsp_cfg);
    
  /* Enable all of them. They are automatically turned off during hibernate mode to save power */
  if(AppCondCfg.BiasVolt == 0.0f)
    AD5940_AFECtrlS(AFECTRL_HSTIAPWR|AFECTRL_INAMPPWR|AFECTRL_EXTBUFPWR|\
                AFECTRL_WG|AFECTRL_DACREFPWR|AFECTRL_HSDACPWR|\
                AFECTRL_SINC2NOTCH, bTRUE);
  else
    AD5940_AFECtrlS(AFECTRL_HSTIAPWR|AFECTRL_INAMPPWR|AFECTRL_EXTBUFPWR|\
                AFECTRL_WG|AFECTRL_DACREFPWR|AFECTRL_HSDACPWR|\
                AFECTRL_SINC2NOTCH|AFECTRL_DCBUFPWR, bTRUE);
    /* Sequence end. */
  /* 序列结束。 */
  AD5940_SEQGenInsert(SEQ_STOP()); /* 添加一个额外的命令来禁用初始化序列的序列发生器，因为我们只想让它运行一次。 */

  /* 在此处停止 */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* 停止序列发生器 */
  if(error == AD5940ERR_OK)
  {
    AppCondCfg.InitSeqInfo.SeqId = SEQID_1;
    AppCondCfg.InitSeqInfo.SeqRamAddr = AppCondCfg.SeqStartAddr;
    AppCondCfg.InitSeqInfo.pSeqCmd = pSeqCmd;
    AppCondCfg.InitSeqInfo.SeqLen = SeqLen;
    /* 将命令写入 SRAM */
    AD5940_SEQCmdWrite(AppCondCfg.InitSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
  }
  else
    return error; /* 错误 */
  return AD5940ERR_OK;
}

static AD5940Err AppCondSeqMeasureGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  uint32_t const *pSeqCmd;
  uint32_t SeqLen;

  uint32_t WaitClks;
  SWMatrixCfg_Type sw_cfg;
  ClksCalInfo_Type clks_cal;

  clks_cal.DataType = DATATYPE_DFT;
  clks_cal.DftSrc = AppCondCfg.DftSrc;
  clks_cal.DataCount = 1L<<(AppCondCfg.DftNum+2); /* 2^(DFTNUMBER+2) */
  clks_cal.ADCSinc2Osr = AppCondCfg.ADCSinc2Osr;
  clks_cal.ADCSinc3Osr = AppCondCfg.ADCSinc3Osr;
  clks_cal.ADCAvgNum = AppCondCfg.ADCAvgNum;
  clks_cal.RatioSys2AdcClk = AppCondCfg.SysClkFreq/AppCondCfg.AdcClkFreq;
  AD5940_ClksCalculate(&clks_cal, &WaitClks);

  /* 在此处启动序列发生器 */
  AD5940_SEQGenCtrl(bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(16*250));  /* @todo wait 250us? */
  sw_cfg.Dswitch = SWD_RCAL0;
  sw_cfg.Pswitch = SWP_RCAL0;
  sw_cfg.Nswitch = SWN_RCAL1;
  sw_cfg.Tswitch = SWT_RCAL1|SWT_TRTIA; 
  AD5940_SWMatrixCfgS(&sw_cfg);  //调整开关矩阵
	AD5940_AFECtrlS(AFECTRL_HSTIAPWR|AFECTRL_INAMPPWR|AFECTRL_EXTBUFPWR|\
                AFECTRL_WG|AFECTRL_DACREFPWR|AFECTRL_HSDACPWR|\
                AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_AFECtrlS(AFECTRL_WG|AFECTRL_ADCPWR, bTRUE);  /* Enable Waveform generator */
  //delay for signal settling DFT_WAIT
  AD5940_SEQGenInsert(SEQ_WAIT(16*800));
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT, bTRUE);  /* Start ADC convert and DFT */
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));
  //wait for first data ready
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_ADCCNV|AFECTRL_DFT, bFALSE);  /* Stop ADC convert and DFT */

  /* Configure matrix for external Rz */
  sw_cfg.Dswitch = AppCondCfg.DswitchSel;
  sw_cfg.Pswitch = AppCondCfg.PswitchSel;
  sw_cfg.Nswitch = AppCondCfg.NswitchSel;
  sw_cfg.Tswitch = SWT_TRTIA|AppCondCfg.TswitchSel;
  AD5940_SWMatrixCfgS(&sw_cfg);
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_WG, bTRUE);  /* Enable Waveform generator */
  AD5940_SEQGenInsert(SEQ_WAIT(16*800));  //delay for signal settling DFT_WAIT
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT, bTRUE);  /* Start ADC convert and DFT */
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));  /* wait for first data ready */
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT|AFECTRL_ADCPWR, bFALSE);  /* Stop ADC convert and DFT */
  AD5940_AFECtrlS(AFECTRL_HSTIAPWR|AFECTRL_INAMPPWR|AFECTRL_EXTBUFPWR|\
                AFECTRL_DACREFPWR|AFECTRL_HSDACPWR|\
                AFECTRL_SINC2NOTCH, bFALSE);

  AD5940_SEQGpioCtrlS(0);        /* Clr GPIO1 */
  AD5940_EnterSleepS();/* 进入休眠 */
  /* 序列结束。 */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* 停止序列发生器 */

  if(error == AD5940ERR_OK)
  {
    AppCondCfg.MeasureSeqInfo.SeqId = SEQID_0;
    AppCondCfg.MeasureSeqInfo.SeqRamAddr = AppCondCfg.InitSeqInfo.SeqRamAddr + AppCondCfg.InitSeqInfo.SeqLen ;
    AppCondCfg.MeasureSeqInfo.pSeqCmd = pSeqCmd;
    AppCondCfg.MeasureSeqInfo.SeqLen = SeqLen;
    /* 将命令写入 SRAM */
    AD5940_SEQCmdWrite(AppCondCfg.MeasureSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
  }
  else
    return error; /* 错误 */
  Serial.printf("[DEBUG] SeqLen = %d\n", SeqLen);
  return AD5940ERR_OK;
}

/* 该函数完成应用初始化（参数、序列、FIFO、校准等） */
AD5940Err AppCondInit(uint32_t *pBuffer, uint32_t BufferSize)
{
  AD5940Err error = AD5940ERR_OK;
  SEQCfg_Type seq_cfg;
  FIFOCfg_Type fifo_cfg;

  if(AD5940_WakeUp(10) > 10)  /* 通过读寄存器唤醒 AFE，最多读取 10 次 */
    return AD5940ERR_WAKEUP;  /* 唤醒失败 */

  /* 配置序列发生器并停止它 */
  seq_cfg.SeqMemSize = SEQMEMSIZE_2KB;  /* 2kB SRAM 用于序列发生器，其余用于数据 FIFO */
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bTRUE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);

  /* 重新配置 FIFO */
  AD5940_FIFOCtrlS(FIFOSRC_DFT, bFALSE);									/* 首先禁用 FIFO */
  fifo_cfg.FIFOEn = bTRUE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;                       /* 4kB 用于 FIFO，其余 2kB 用于序列发生器 */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = AppCondCfg.FifoThresh;              /* DFT 结果。一对用于 RCAL，另一对用于 Rz。一个 DFT 结果包含实部和虚部 */
  AD5940_FIFOCfg(&fifo_cfg);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  /* 启动序列发生器 */
  /* 初始化序列发生器 */
  if((AppCondCfg.CondInited == bFALSE)||\
    (AppCondCfg.bParaChanged == bTRUE))
  {
    if(pBuffer == 0)  return AD5940ERR_PARA;
    if(BufferSize == 0) return AD5940ERR_PARA;
    AD5940_SEQGenInit(pBuffer, BufferSize);

    /* 生成初始化序列 */
    error = AppCondSeqCfgGen(); /* 使用 MCU 或序列发生器的应用程序初始化序列 */
    if(error != AD5940ERR_OK) return error;

    /* 生成测量序列 */
    error = AppCondSeqMeasureGen();
    if(error != AD5940ERR_OK) return error;

    AppCondCfg.bParaChanged = bFALSE; /* 清除此标志，因为我们已经实现了新的配置 */
  }

  /* 初始化序列发生器  */
  AppCondCfg.InitSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppCondCfg.InitSeqInfo);
  seq_cfg.SeqEnable = bTRUE;
  AD5940_SEQCfg(&seq_cfg);  /* 启用序列发生器 */
  AD5940_SEQMmrTrig(AppCondCfg.InitSeqInfo.SeqId);
  while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_ENDSEQ) == bFALSE);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  /* 测量序列  */
  AppCondCfg.MeasureSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppCondCfg.MeasureSeqInfo);

  seq_cfg.SeqEnable = bTRUE;
  AD5940_SEQCfg(&seq_cfg);  /* 启用序列发生器，并等待触发 */
  AD5940_ClrMCUIntFlag();   /* 清除之前生成的中断标志 */
  AD5940_AFEPwrBW(AppCondCfg.PwrMod, AFEBW_250KHZ);

  AppCondCfg.CondInited = bTRUE;  /* Cond 应用程序已初始化。 */
  return AD5940ERR_OK;
}

/* AFE 唤醒时修改寄存器 */
static AD5940Err AppCondRegModify(int32_t * const pData, uint32_t *pDataCount)
{
  if(AppCondCfg.NumOfData > 0)
  {
    AppCondCfg.FifoDataCount += *pDataCount/4;
    if(AppCondCfg.FifoDataCount >= AppCondCfg.NumOfData)
    {
      AD5940_WUPTCtrl(bFALSE);
      return AD5940ERR_OK;
    }
  }
  if(AppCondCfg.StopRequired == bTRUE)
  {
    AD5940_WUPTCtrl(bFALSE);
    return AD5940ERR_OK;
  }
  if(AppCondCfg.SweepCfg.SweepEn) /* 需要设置新频率并设置电源模式 */
  {
    AD5940_WGFreqCtrlS(AppCondCfg.SweepNextFreq, AppCondCfg.SysClkFreq);
  }
  return AD5940ERR_OK;
}

/* 根据数据类型，在返回给控制器之前进行适当的数据预处理 */
static AD5940Err AppCondDataProcess(int32_t * const pData, uint32_t *pDataCount)
{
  uint32_t DataCount = *pDataCount;
  uint32_t ImpResCount = DataCount/4;

  fImpCar_Type * const pOut = (fImpCar_Type*)pData;
  iImpCar_Type * pSrcData = (iImpCar_Type*)pData;

  *pDataCount = 0;

  DataCount = (DataCount/4)*4; /* 一个 DFT 结果在 FIFO 中有两个数据，实部和虚部。每次测量有 2 个 DFT 结果，一个用于电压测量，一个用于电流测量 */

  /* 将 DFT 结果转换为 int32_t 类型 */
  for(uint32_t i=0; i<DataCount; i++)
  {
    pData[i] &= 0x3ffff;
    if(pData[i]&(1<<17)) /* 第 17 位是符号位 */
    {
      pData[i] |= 0xfffc0000; /* 数据是 18 位补码，第 17 位是符号位 */
    }
  }
  for(uint32_t i=0; i<ImpResCount; i++)
  {
    iImpCar_Type *pDftRcal, *pDftRz;
    float RzMag, RzPhase;
    float RcalMag, RcalPhase;

    // 提取 RCAL 和 Rz 的 DFT 数据
    pDftRcal = pSrcData++;
    pDftRz   = pSrcData++;
    
    // 计算 RCAL 和 Rz 的模值 (Magnitude)
    RcalMag = sqrt((float)pDftRcal->Real * pDftRcal->Real + (float)pDftRcal->Image * pDftRcal->Image);
    RzMag   = sqrt((float)pDftRz->Real * pDftRz->Real + (float)pDftRz->Image * pDftRz->Image);
    
    // 计算 RCAL 和 Rz 的相位 (Phase)
    RcalPhase = atan2(-pDftRcal->Image, pDftRcal->Real);
    RzPhase   = atan2(-pDftRz->Image, pDftRz->Real);

    // 比例法计算目标阻抗真正的幅值与相位 (抵消系统误差)
    RzMag   = RcalMag / RzMag * AppCondCfg.RcalVal;
    RzPhase = RcalPhase - RzPhase;

    // --- 关键兼容层 ---
    // 将极坐标 (RzMag, RzPhase) 转换回直角坐标 (Real, Image)，以兼容现有上层状态机调用
    pOut[i].Real  = RzMag * cos(RzPhase);
    pOut[i].Image = RzMag * sin(RzPhase);

  }
  *pDataCount = ImpResCount;
  /* 计算下一个频率点 */
  if(AppCondCfg.SweepCfg.SweepEn == bTRUE)
  {
    AppCondCfg.FreqofData = AppCondCfg.SweepCurrFreq;
    AppCondCfg.SweepCurrFreq = AppCondCfg.SweepNextFreq;
    AD5940_SweepNext(&AppCondCfg.SweepCfg, &AppCondCfg.SweepNextFreq);
  }
  return AD5940ERR_OK;
}


AD5940Err AppCondISR(void *pBuff, uint32_t *pCount)
{
  uint32_t BuffCount;
  uint32_t FifoCnt;
  BuffCount = *pCount;
  *pCount = 0;
  if(AppCondCfg.CondInited == bFALSE)
    return AD5940ERR_APPERROR;
  if(AD5940_WakeUp(10) > 10)  /* 通过读寄存器唤醒 AFE，最多读取 10 次 */
    return AD5940ERR_WAKEUP;  /* 唤醒失败 */

  AD5940_SleepKeyCtrlS(SLPKEY_LOCK);  /* 不要进入休眠 */

  if(AD5940_INTCTestFlag(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH) == bTRUE)
  {
    /* 现在 FIFO 中应该有 4 个数据 */
    FifoCnt = (AD5940_FIFOGetCnt()/4)*4;

    AD5940_FIFORd((uint32_t *)pBuff, FifoCnt);
    AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
    AppCondRegModify(static_cast<int32_t*>(pBuff), &FifoCnt);   /* 如果需要重新配置 AFE，请在 AFE 处于活动状态时在此处执行 */
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* 允许 AFE 进入休眠模式 */
    /* 进行数据处理 */
    AppCondDataProcess((int32_t*)pBuff,&FifoCnt);
    *pCount = FifoCnt;
    return 0;
  }

  return 0;
}
float ComputeKCell(uint32_t *pData, uint32_t DataCount)
{ 
  float freq;
  fImpCar_Type *pImp = (fImpCar_Type*)pData;
  AppCondCtrl(CondCTRL_GETFREQ, &freq);
  for(int i=0; i<DataCount; i++)
  {
    float R = pImp[i].Real;   // 阻抗实部 (Resistance)
    float X = pImp[i].Image;  // 阻抗虚部 (Reactance)
    
    // 1. 计算阻抗模的平方: |Z|^2 = R^2 + X^2
    float magSq = R*R + X*X;
    
    float Conductance_S = 0.0f; // 单位：西门子 (Siemens)
    
    // 2. 计算电导 (Conductance, G). 
    // 公式: Y = 1/Z = (R - jX) / (R^2 + X^2)
    // 电导 G = Y的实部 = R / (R^2 + X^2)
    if(magSq > 0.0f)
    {
        Conductance_S = R / magSq;
    }
    // 3. 转换为微西门子 (uS)，因为自来水的电导通常在 uS 级别
    float Conductance_uS = Conductance_S * 1e6f;
    return Conductance_uS;
  }
  return 0.0f;
}
/* 修改后的显示函数，增加电导计算，支持单次测量和扫频两种输出格式 */
int32_t CondShowResult(uint32_t *pData, uint32_t DataCount, bool isSweep, int sweepIndex, int sweepTotal)
{
  float freq;
  fImpCar_Type *pImp = (fImpCar_Type*)pData;
  AppCondCtrl(CondCTRL_GETFREQ, &freq);
  for(int i=0; i<DataCount; i++)
  {
    float R = pImp[i].Real;
    float X = pImp[i].Image;
    float magSq = R*R + X*X;
    float Conductance_S = 0.0f;
    if(magSq > 0.0f)
    {
        Conductance_S = R / magSq;
    }
    float Conductance_uS = Conductance_S * 1e6f;
    float Conductivity_uS_cm = ApplyCondCalib(Conductance_uS * AppCondCfg.K_Cell);
    float mag   = AD5940_ComplexMag(&pImp[i]);
    float phase = AD5940_ComplexPhase(&pImp[i]) * 180.0f / MATH_PI;

    if(isSweep)
    {
      // $COND,SWEEP,<point_index>,<total_points>,<freq_Hz>,<mag_Ohm>,<phase_deg>,<real_Ohm>,<imag_Ohm>,<G_uS>,<conductivity_uS_cm>*
      Serial.printf("$COND,SWEEP,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f*\n",
                    sweepIndex, sweepTotal,
                    freq, mag, phase, R, X,
                    Conductance_uS, Conductivity_uS_cm);
    }
    else
    {
      // $COND,MEAS,<freq_Hz>,<mag_Ohm>,<phase_deg>,<real_Ohm>,<imag_Ohm>,<G_uS>,<conductivity_uS_cm>*
      Serial.printf("$COND,MEAS,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f*\n",
                    freq, mag, phase, R, X,
                    Conductance_uS, Conductivity_uS_cm);
    }
  }
  return 0;
}

// ================= 工具函数：解 3x3 线性方程组 (高斯消元) =================
static inline bool condSolve3x3(float A[3][3], float y[3], float x[3]) {
  for (int i = 0; i < 3; ++i) {
    float pivot = A[i][i];
    if (fabsf(pivot) < 1e-12f) return false; // 主元过小，无解
    float inv = 1.0f / pivot;
    for (int j = i; j < 3; ++j) A[i][j] *= inv;
    y[i] *= inv;
    for (int r = 0; r < 3; ++r) if (r != i) {
      float f = A[r][i];
      for (int c = i; c < 3; ++c) A[r][c] -= f * A[i][c];
      y[r] -= f * y[i];
    }
  }
  x[0] = y[0]; x[1] = y[1]; x[2] = y[2];
  return true;
}

// ================= 三点校准函数实现 =================
bool CondFitThreePoint(const CondCalibPoint& p1,
                       const CondCalibPoint& p2,
                       const CondCalibPoint& p3,
                       CondCalibCoeff& coeff)
{
  // 防护：如果三点的 cond_meas 差值 < 10.0f，返回 false
  if (fabsf(p1.cond_meas - p2.cond_meas) < 10.0f ||
      fabsf(p2.cond_meas - p3.cond_meas) < 10.0f ||
      fabsf(p1.cond_meas - p3.cond_meas) < 10.0f) {
    Serial.println("[COND_CAL] Warning: Points too close, cannot fit!");
    return false;
  }

  // 我们要解方程组: cond_true = a*cond_meas^2 + b*cond_meas + c
  // 构建矩阵方程: [g^2  g  1] * [a b c]^T = cond_true

  float A[3][3] = {
    { p1.cond_meas * p1.cond_meas, p1.cond_meas, 1.0f },
    { p2.cond_meas * p2.cond_meas, p2.cond_meas, 1.0f },
    { p3.cond_meas * p3.cond_meas, p3.cond_meas, 1.0f }
  };

  float Y[3] = { p1.cond_true, p2.cond_true, p3.cond_true };
  float X[3] = { 0, 0, 0 };

  if (!condSolve3x3(A, Y, X))
  {
    Serial.println("[COND_CAL] Matrix solve failed!");
    return false;
  }

  coeff.a = X[0];
  coeff.b = X[1];
  coeff.c = X[2];
  coeff.valid = true;
  return true;
}

// ================= 应用校准系数 =================
float ApplyCondCalib(float raw_conductivity)
{
  if (!g_condCalib.valid) {
    return raw_conductivity; // 如果没有校准，直接返回原始值
  }
  return g_condCalib.a * raw_conductivity * raw_conductivity
       + g_condCalib.b * raw_conductivity
       + g_condCalib.c;
}

/**
 * @brief  根据当前温度，返回 Apera 1413 µS/cm 标液的真实电导率
 * @param  temp_C  PT1000测量温度 (°C)
 * @return 真实电导率 (µS/cm)
 * @note   拟合公式 y = 0.1002*T^2 + 22.735*T + 781.69，
 *         有效范围约 5~40°C
 */
float ApecaStd1413_TrueEC(float temp_C)
{
    return 0.1002f * temp_C * temp_C
         + 22.735f * temp_C
         + 781.69f;
}