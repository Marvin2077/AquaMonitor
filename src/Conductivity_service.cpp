#include "Conductivity_service.h"
#include "Arduino.h"

AppCondCfg_Type AppCondCfg;

AD5940Err AppCondCfg_init(){

    AppCondCfg.bParaChanged = bFALSE;
    AppCondCfg.SeqStartAddr = 0;
    AppCondCfg.MaxSeqLen = 512;
    AppCondCfg.SeqStartAddrCal = 0;
    AppCondCfg.MaxSeqLenCal = 0;

    AppCondCfg.ReDoRtiaCal = bFALSE;
    AppCondCfg.SysClkFreq = 16000000.0;
    AppCondCfg.WuptClkFreq = 32000.0;
    AppCondCfg.AdcClkFreq = 16000000.0;
    AppCondCfg.CondODR = 20.0; /* 20.0 赫兹 */
    AppCondCfg.NumOfData = -1;
    AppCondCfg.RcalVal = 1000.0; /* 10k 欧姆 */

    AppCondCfg.PwrMod = AFEPWR_LP;
    AppCondCfg.HstiaRtiaSel = HSTIARTIA_1K;
    AppCondCfg.CtiaSel = 16;
    AppCondCfg.ExcitBufGain = EXCITBUFGAIN_2;
    AppCondCfg.HsDacGain = HSDACGAIN_1;
    AppCondCfg.HsDacUpdateRate = 7;
    AppCondCfg.DacVoltPP = 600.0;

    AppCondCfg.SinFreq = 50000.0; /* 50000 赫兹 */

    AppCondCfg.ADCPgaGain = ADCPGA_1P5;
    AppCondCfg.ADCSinc3Osr = ADCSINC3OSR_2;
    AppCondCfg.ADCSinc2Osr = ADCSINC2OSR_22;

    AppCondCfg.DftNum = DFTNUM_8192;
    AppCondCfg.DftSrc = DFTSRC_SINC3;
    AppCondCfg.HanWinEn = bTRUE;

    AppCondCfg.DswitchSel = SWD_CE0;
    AppCondCfg.PswitchSel = SWP_CE0;
    AppCondCfg.NswitchSel = SWN_AIN0;
    AppCondCfg.TswitchSel = SWN_AIN0;

    AppCondCfg.SweepCfg.SweepEn = bFALSE;
    AppCondCfg.SweepCfg.SweepStart = 2000.0;
    AppCondCfg.SweepCfg.SweepStop = 200000.0;
    AppCondCfg.SweepCfg.SweepPoints = 100;
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

/* 生成初始化序列 */
static AD5940Err AppCondSeqCfgGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  uint32_t const *pSeqCmd;
  uint32_t SeqLen;

  AFERefCfg_Type aferef_cfg;
  HSLoopCfg_Type hs_loop;
  DSPCfg_Type dsp_cfg;
  float sin_freq;

  /* 在此处启动序列发生器 */
  AD5940_SEQGenCtrl(bTRUE);

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
  aferef_cfg.LpBandgapEn = bTRUE;
  aferef_cfg.LpRefBufEn = bTRUE;
  aferef_cfg.LpRefBoostEn = bFALSE;
  AD5940_REFCfgS(&aferef_cfg);
  AD5940_StructInit(&hs_loop,sizeof(hs_loop));
  hs_loop.HsDacCfg.ExcitBufGain = AppCondCfg.ExcitBufGain;
  hs_loop.HsDacCfg.HsDacGain = AppCondCfg.HsDacGain;
  hs_loop.HsDacCfg.HsDacUpdateRate = AppCondCfg.HsDacUpdateRate;

  hs_loop.HsTiaCfg.DiodeClose = bFALSE;
  hs_loop.HsTiaCfg.HstiaBias = HSTIABIAS_1P1;
  hs_loop.HsTiaCfg.HstiaCtia = AppCondCfg.CtiaSel; /* 31pF + 2pF */
  hs_loop.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
  hs_loop.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_OPEN;
  hs_loop.HsTiaCfg.HstiaRtiaSel = AppCondCfg.HstiaRtiaSel;

  hs_loop.SWMatCfg.Dswitch = SWD_OPEN;
  hs_loop.SWMatCfg.Pswitch = SWP_PL|SWP_PL2;
  hs_loop.SWMatCfg.Nswitch = SWN_NL|SWN_NL2;
  hs_loop.SWMatCfg.Tswitch = SWT_AIN1 |SWT_TRTIA;

  hs_loop.WgCfg.WgType = WGTYPE_SIN;
  hs_loop.WgCfg.GainCalEn = bFALSE;
  hs_loop.WgCfg.OffsetCalEn = bFALSE;
  if(AppCondCfg.SweepCfg.SweepEn == bTRUE)
  {
    AppCondCfg.SweepCfg.SweepIndex = 0;
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
  hs_loop.WgCfg.SinCfg.SinFreqWord = AD5940_WGFreqWordCal(sin_freq, AppCondCfg.SysClkFreq);
  hs_loop.WgCfg.SinCfg.SinAmplitudeWord = (uint32_t)(AppCondCfg.DacVoltPP/800.0f*2047 + 0.5f);
  hs_loop.WgCfg.SinCfg.SinOffsetWord = 0;
  hs_loop.WgCfg.SinCfg.SinPhaseWord = 0;
  AD5940_HSLoopCfgS(&hs_loop);

  AD5940_StructInit(&dsp_cfg,sizeof(dsp_cfg));
  dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_HSTIA_N;
  dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_HSTIA_P;
  dsp_cfg.ADCBaseCfg.ADCPga = AppCondCfg.ADCPgaGain;

  memset(&dsp_cfg.ADCDigCompCfg, 0, sizeof(dsp_cfg.ADCDigCompCfg));

  dsp_cfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_16;  /* 此设置无关紧要（该功能未启用） */
  dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ;	/* 告诉滤波链当前 ADC 时钟速率 */
  dsp_cfg.ADCFilterCfg.ADCSinc2Osr = AppCondCfg.ADCSinc2Osr;
  dsp_cfg.ADCFilterCfg.ADCSinc3Osr = AppCondCfg.ADCSinc3Osr;
  dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE;
  dsp_cfg.ADCFilterCfg.BpNotch = bTRUE;
  dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;
  dsp_cfg.DftCfg.DftNum = AppCondCfg.DftNum;
  dsp_cfg.DftCfg.DftSrc = AppCondCfg.DftSrc;
  dsp_cfg.DftCfg.HanWinEn = AppCondCfg.HanWinEn;

  memset(&dsp_cfg.StatCfg, 0, sizeof(dsp_cfg.StatCfg)); /* 不关心统计数据 */
  AD5940_DSPCfgS(&dsp_cfg);

  /* 启用所有这些。它们在休眠模式下会自动关闭以节省功耗 */
  AD5940_AFECtrlS(AFECTRL_HPREFPWR|AFECTRL_HSTIAPWR|AFECTRL_INAMPPWR|AFECTRL_EXTBUFPWR|\
    AFECTRL_DACREFPWR|AFECTRL_HSDACPWR|\
      AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_SEQGpioCtrlS(0/*AGPIO_Pin6|AGPIO_Pin5|AGPIO_Pin1*/);        //GP6->结束序列, GP5 -> AD8233=关闭, GP1->RLD=关闭 .

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
  clks_cal.ADCAvgNum = 0;
  clks_cal.RatioSys2AdcClk = AppCondCfg.SysClkFreq/AppCondCfg.AdcClkFreq;
  AD5940_ClksCalculate(&clks_cal, &WaitClks);

  /* 在此处启动序列发生器 */
  AD5940_SEQGenCtrl(bTRUE);
  AD5940_SEQGpioCtrlS(AGPIO_Pin1/*|AGPIO_Pin5|AGPIO_Pin1*/);//GP6->结束序列, GP5 -> AD8233=关闭, GP1->RLD=关闭 .

	/* 配置开关矩阵以连接传感器 */
  sw_cfg.Dswitch = AppCondCfg.DswitchSel;
  sw_cfg.Pswitch = AppCondCfg.PswitchSel;
  sw_cfg.Nswitch = AppCondCfg.NswitchSel;
  sw_cfg.Tswitch = AppCondCfg.TswitchSel|SWT_TRTIA;
  AD5940_SWMatrixCfgS(&sw_cfg);

  AD5940_SEQGenInsert(SEQ_WAIT(16*250));
  /* 步骤 1：测量电流 */
  AD5940_ADCMuxCfgS(ADCMUXP_HSTIA_P, ADCMUXN_HSTIA_N);
  AD5940_AFECtrlS(AFECTRL_WG|AFECTRL_ADCPWR, bTRUE);  /* 启用波形发生器, ADC 电源 */
  AD5940_SEQGenInsert(SEQ_WAIT(16*80));
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT, bTRUE);  /* 启动 ADC 转换和 DFT */
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));  /* 等待第一个数据就绪 */
	AD5940_SEQGenInsert(SEQ_WAIT(1));
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT|AFECTRL_WG|AFECTRL_ADCPWR, bFALSE);  /* 停止 ADC 转换和 DFT */

  /* 步骤 2：测量电压 */
  AD5940_ADCMuxCfgS(ADCMUXP_VCE0, ADCMUXN_N_NODE);
  AD5940_AFECtrlS(AFECTRL_WG|AFECTRL_ADCPWR, bTRUE);  /* 启用波形发生器, ADC 电源 */
  AD5940_SEQGenInsert(SEQ_WAIT(16*80));
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT, bTRUE);  /* 启动 ADC 转换和 DFT */
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));  /* 等待第一个数据就绪 */
	AD5940_SEQGenInsert(SEQ_WAIT(1));
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT|AFECTRL_WG|AFECTRL_ADCPWR, bFALSE);  /* 停止 ADC 转换和 DFT */

  sw_cfg.Dswitch = SWD_OPEN;
  sw_cfg.Pswitch = SWP_PL|SWP_PL2;
  sw_cfg.Nswitch = SWN_NL|SWN_NL2;
  sw_cfg.Tswitch = SWT_TRTIA;
  AD5940_SWMatrixCfgS(&sw_cfg); /* 浮空开关 */

  AD5940_SEQGpioCtrlS(0/*AGPIO_Pin6|AGPIO_Pin5|AGPIO_Pin1*/);        //GP6->结束序列, GP5 -> AD8233=关闭, GP1->RLD=关闭 .
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
  return AD5940ERR_OK;
}

static AD5940Err AppCondRtiaCal(void)
{
  HSRTIACal_Type hsrtia_cal;
  FreqParams_Type freq_params;

  if(AppCondCfg.SweepCfg.SweepEn == bTRUE)
  {
    hsrtia_cal.fFreq = AppCondCfg.SweepCfg.SweepStart;
    freq_params = AD5940_GetFreqParameters(AppCondCfg.SweepCfg.SweepStart);
  }
  else
  {
    hsrtia_cal.fFreq = AppCondCfg.SinFreq;
    freq_params = AD5940_GetFreqParameters(AppCondCfg.SinFreq);
  }

  if(freq_params.HighPwrMode == bTRUE)
    hsrtia_cal.AdcClkFreq = 32e6;
  else
    hsrtia_cal.AdcClkFreq = 16e6;

  hsrtia_cal.ADCSinc2Osr = freq_params.ADCSinc2Osr;
  hsrtia_cal.ADCSinc3Osr = freq_params.ADCSinc3Osr;
  hsrtia_cal.DftCfg.DftNum = freq_params.DftNum;
  hsrtia_cal.DftCfg.DftSrc = freq_params.DftSrc;
  hsrtia_cal.bPolarResult = bTRUE; /* 我们在这里需要幅度和相位 */
  hsrtia_cal.DftCfg.HanWinEn = AppCondCfg.HanWinEn;
  hsrtia_cal.fRcal= AppCondCfg.RcalVal;
  hsrtia_cal.HsTiaCfg.DiodeClose = bFALSE;
  hsrtia_cal.HsTiaCfg.HstiaBias = HSTIABIAS_1P1;
  hsrtia_cal.HsTiaCfg.HstiaCtia = AppCondCfg.CtiaSel;
  hsrtia_cal.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
  hsrtia_cal.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_OPEN;
  hsrtia_cal.HsTiaCfg.HstiaRtiaSel = AppCondCfg.HstiaRtiaSel;
  hsrtia_cal.SysClkFreq = AppCondCfg.SysClkFreq;


  if(AppCondCfg.SweepCfg.SweepEn == bTRUE)
  {
    uint32_t i;
    AppCondCfg.SweepCfg.SweepIndex = 0;  /* 复位扫频索引 */
    for(i=0;i<AppCondCfg.SweepCfg.SweepPoints;i++)
    {
      AD5940_HSRtiaCal(&hsrtia_cal, &AppCondCfg.RtiaCalTable[i]);
#ifdef ADI_DEBUG
      ADI_Print("Freq:%.2f, (%f, %f)Ohm\n", hsrtia_cal.fFreq, AppCondCfg.RtiaCalTable[i].Real, AppCondCfg.RtiaCalTable[i].Image);
#endif
      AD5940_SweepNext(&AppCondCfg.SweepCfg, &hsrtia_cal.fFreq);
      freq_params = AD5940_GetFreqParameters(hsrtia_cal.fFreq);

      if(freq_params.HighPwrMode == bTRUE)
      {
        hsrtia_cal.AdcClkFreq = 32e6;
        /* 将时钟切换到 32MHz 振荡器 */
        AD5940_HPModeEn(bTRUE);
      }
      else
      {
        hsrtia_cal.AdcClkFreq = 16e6;
        /* 将时钟切换为 16MHz 振荡器 */
	      AD5940_HPModeEn(bFALSE);
      }
      hsrtia_cal.ADCSinc2Osr = freq_params.ADCSinc2Osr;
      hsrtia_cal.ADCSinc3Osr = freq_params.ADCSinc3Osr;
      hsrtia_cal.DftCfg.DftNum = freq_params.DftNum;
      hsrtia_cal.DftCfg.DftSrc = freq_params.DftSrc;
    }

    AppCondCfg.SweepCfg.SweepIndex = 0;  /* 复位扫频索引 */
    AppCondCfg.RtiaCurrValue = AppCondCfg.RtiaCalTable[0];
  }
  else
  {
    AD5940_HSRtiaCal(&hsrtia_cal, &AppCondCfg.RtiaCurrValue);
#ifdef ADI_DEBUG
    ADI_Print("Freq:%.2f, (%f, %f)Ohm\n", hsrtia_cal.fFreq, AppCondCfg.RtiaCurrValue.Real, AppCondCfg.RtiaCurrValue.Image);
#endif
  }
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
  seq_cfg.SeqIgnoreEn = bFALSE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);

  /* 执行 RTIA 校准 */
  if((AppCondCfg.ReDoRtiaCal == bTRUE) || \
    AppCondCfg.CondInited == bFALSE)  /* 在第一次初始化时执行校准 */
  {
    AppCondRtiaCal();
    AppCondCfg.ReDoRtiaCal = bFALSE;
  }
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

  AppCondCheckFreq(AppCondCfg.FreqofData);
  seq_cfg.SeqEnable = bTRUE;
  AD5940_SEQCfg(&seq_cfg);  /* 启用序列发生器，并等待触发 */
  AD5940_ClrMCUIntFlag();   /* 清除之前生成的中断标志 */

  AppCondCfg.CondInited = bTRUE;  /* Cond 应用程序已初始化。 */
  return AD5940ERR_OK;
}

/* 根据正弦波的频率设置最佳滤波器设置 */
AD5940Err AppCondCheckFreq(float freq)
{
  ADCFilterCfg_Type filter_cfg;
  DFTCfg_Type dft_cfg;
  HSDACCfg_Type hsdac_cfg;
  uint32_t WaitClks;
  ClksCalInfo_Type clks_cal;
  FreqParams_Type freq_params;
  uint32_t SeqCmdBuff[2];
  uint32_t SRAMAddr = 0;;
  /* 步骤 1：检查频率 */
  freq_params = AD5940_GetFreqParameters(freq);

	/* 设置电源模式 */
  if(freq_params.HighPwrMode == bTRUE)
  {
    /* 更新 HSDAC 更新速率 */
    hsdac_cfg.ExcitBufGain = AppCondCfg.ExcitBufGain;
    hsdac_cfg.HsDacGain = AppCondCfg.HsDacGain;
    hsdac_cfg.HsDacUpdateRate = 0x7;
    AD5940_HSDacCfgS(&hsdac_cfg);

    /*更新 ADC 速率 */
    filter_cfg.ADCRate = ADCRATE_1P6MHZ;
    AppCondCfg.AdcClkFreq = 32e6;

    /* 将时钟切换到 32MHz 振荡器 */
    AD5940_HPModeEn(bTRUE);
  }else
	{
    /* 更新 HSDAC 更新速率 */
    hsdac_cfg.ExcitBufGain = AppCondCfg.ExcitBufGain;
    hsdac_cfg.HsDacGain = AppCondCfg.HsDacGain;
    hsdac_cfg.HsDacUpdateRate = 0x1B;
    AD5940_HSDacCfgS(&hsdac_cfg);
    /* 更新 ADC 速率 */
    filter_cfg.ADCRate = ADCRATE_800KHZ;
    AppCondCfg.AdcClkFreq = 16e6;

		/* 将时钟切换为 16MHz 振荡器 */
    AD5940_HPModeEn(bFALSE);
  }

  /* 步骤 2：调整 ADCFILTERCON 和 DFTCON 以设置最佳的 SINC3、SINC2 和 DFTNUM 设置 */
  filter_cfg.ADCAvgNum = ADCAVGNUM_16;  /* 不关心，因为它被禁用了 */
  filter_cfg.ADCSinc2Osr = freq_params.ADCSinc2Osr;
  filter_cfg.ADCSinc3Osr = freq_params.ADCSinc3Osr;
  filter_cfg.BpSinc3 = bFALSE;
  filter_cfg.BpNotch = bTRUE;
  filter_cfg.Sinc2NotchEnable = bTRUE;
  dft_cfg.DftNum = freq_params.DftNum;
  dft_cfg.DftSrc = freq_params.DftSrc;
  dft_cfg.HanWinEn = AppCondCfg.HanWinEn;
  AD5940_ADCFilterCfgS(&filter_cfg);
  AD5940_DFTCfgS(&dft_cfg);

  /* 步骤 3：计算获取结果到 FIFO 所需的时钟，并更新序列发生器等待命令 */
  clks_cal.DataType = DATATYPE_DFT;
  clks_cal.DftSrc = freq_params.DftSrc;
  clks_cal.DataCount = 1L<<(freq_params.DftNum+2); /* 2^(DFTNUMBER+2) */
  clks_cal.ADCSinc2Osr = freq_params.ADCSinc2Osr;
  clks_cal.ADCSinc3Osr = freq_params.ADCSinc3Osr;
  clks_cal.ADCAvgNum = 0;
  clks_cal.RatioSys2AdcClk = AppCondCfg.SysClkFreq/AppCondCfg.AdcClkFreq;
  AD5940_ClksCalculate(&clks_cal, &WaitClks);

	/* 最大时钟数是 0x3FFFFFFF。如果频率很低，则需要更多时钟 */
	if(WaitClks > 0x3FFFFFFF)
	{
		WaitClks /=2;
		SRAMAddr = AppCondCfg.MeasureSeqInfo.SeqRamAddr;
		SeqCmdBuff[0] = SEQ_WAIT(WaitClks);
		AD5940_SEQCmdWrite(SRAMAddr+11, SeqCmdBuff, 1);
		AD5940_SEQCmdWrite(SRAMAddr+12, SeqCmdBuff, 1);
		AD5940_SEQCmdWrite(SRAMAddr+18, SeqCmdBuff, 1);
		AD5940_SEQCmdWrite(SRAMAddr+19, SeqCmdBuff, 1);
	}
	else
	{
		SRAMAddr = AppCondCfg.MeasureSeqInfo.SeqRamAddr;
		SeqCmdBuff[0] = SEQ_WAIT(WaitClks);
		AD5940_SEQCmdWrite(SRAMAddr+11, SeqCmdBuff, 1);
		AD5940_SEQCmdWrite(SRAMAddr+18, SeqCmdBuff, 1);
	}

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
    AppCondCheckFreq(AppCondCfg.SweepNextFreq);
    AD5940_WGFreqCtrlS(AppCondCfg.SweepNextFreq, AppCondCfg.SysClkFreq);
  }
  return AD5940ERR_OK;
}

/* 根据数据类型，在返回给控制器之前进行适当的数据预处理 */
static AD5940Err AppCondDataProcess(int32_t * const pData, uint32_t *pDataCount)
{
  uint32_t DataCount = *pDataCount;
  uint32_t ImpResCount = DataCount/4;

  fImpCar_Type * pOut = (fImpCar_Type*)pData;
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
    fImpCar_Type DftCurr, DftVolt;
    fImpCar_Type res;

    DftCurr.Real = (float)pSrcData[i].Real;
    DftCurr.Image = (float)pSrcData[i].Image;
    DftVolt.Real = (float)pSrcData[i+1].Real;
    DftVolt.Image = (float)pSrcData[i+1].Image;

    DftCurr.Real = -DftCurr.Real;
    DftCurr.Image = -DftCurr.Image;
    DftVolt.Real = DftVolt.Real;
    DftVolt.Image = DftVolt.Image;
    res = AD5940_ComplexDivFloat(&DftCurr, &AppCondCfg.RtiaCurrValue);           /* I=Vrtia/Zrtia */
    res = AD5940_ComplexDivFloat(&DftVolt, &res);
    pOut[i] = res;
  }
  *pDataCount = ImpResCount;
  /* 计算下一个频率点 */
  if(AppCondCfg.SweepCfg.SweepEn == bTRUE)
  {
    AppCondCfg.FreqofData = AppCondCfg.SweepCurrFreq;
    AppCondCfg.SweepCurrFreq = AppCondCfg.SweepNextFreq;
    AppCondCfg.RtiaCurrValue = AppCondCfg.RtiaCalTable[AppCondCfg.SweepCfg.SweepIndex];
    AD5940_SweepNext(&AppCondCfg.SweepCfg, &AppCondCfg.SweepNextFreq);
  }
  return AD5940ERR_OK;
}

/**

*/
AD5940Err AppCondISR(void *pBuff, uint32_t *pCount)
{
  uint32_t BuffCount;
  uint32_t FifoCnt;
  if(AppCondCfg.CondInited == bFALSE)
    return AD5940ERR_APPERROR;
  if(AD5940_WakeUp(10) > 10)  /* 通过读寄存器唤醒 AFE，最多读取 10 次 */
    return AD5940ERR_WAKEUP;  /* 唤醒失败 */

  AD5940_SleepKeyCtrlS(SLPKEY_LOCK);  /* 不要进入休眠 */
  *pCount = 0;

  if(AD5940_INTCTestFlag(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH) == bTRUE)
  {
    /* 现在 FIFO 中应该有 4 个数据 */
    FifoCnt = (AD5940_FIFOGetCnt()/4)*4;
    AD5940_FIFORd((uint32_t *)pBuff, FifoCnt);
    AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
    AppCondRegModify(static_cast<int32_t*>(pBuff), &FifoCnt);   /* 如果需要重新配置 AFE，请在 AFE 处于活动状态时在此处执行 */
    AD5940_EnterSleepS();  /* 手动让 AFE 返回休眠以节省功耗 */
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

}
/* 修改后的显示函数，增加电导计算 */
int32_t CondShowResult(uint32_t *pData, uint32_t DataCount)
{
  float freq;

  // 电导率 (S/m) = 电导 (S) * K (1/m)
  fImpCar_Type *pImp = (fImpCar_Type*)pData;
  AppCondCtrl(CondCTRL_GETFREQ, &freq);
  /* Process data */
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

    // 4. 计算电导率 (Conductivity)，需要乘以电极常数 K
    float Conductivity_uS_cm = Conductance_uS * AppCondCfg.K_Cell;

    Serial.printf("Freq: %.2f, RzMag: %.2f Ohm, Phase: %.2f, Real=%.2f, Image=%.2f, G: %.4f uS ", 
                  freq, 
                  AD5940_ComplexMag(&pImp[i]), 
                  AD5940_ComplexPhase(&pImp[i])*180/MATH_PI, 
                  R, X,
                  Conductance_uS);
    Serial.printf("Conductivity %f\n",Conductivity_uS_cm);
    /* 如果你需要查看原本的实部虚部，可以取消下面注释 */
    // Serial.printf("Debug: Real=%.2f, Image=%.2f\n", R, X);
  }
  return 0;
}