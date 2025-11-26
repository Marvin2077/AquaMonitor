#include "ph_service.h"
#include "Arduino.h"

AppPHCfg_Type AppPHCfg;

AD5940Err AppPHCfg_init()
{
    AppPHCfg.bParaChanged = bFALSE;
    AppPHCfg.SeqStartAddr = 0;
    AppPHCfg.MaxSeqLen = 512;
    AppPHCfg.SeqStartAddrCal = 0;
    AppPHCfg.MaxSeqLenCal = 0;
    //LPDAC
    AppPHCfg.LpdacSel = LPDAC0;
    // 设置 VBIAS = 1.2V (0x745)
    AppPHCfg.DacData12Bit = 0x745;
    // 设置 VZERO = 1.0V (0x17)
    AppPHCfg.DacData6Bit = 0x17;
    if (AppPHCfg.DacData12Bit > (AppPHCfg.DacData6Bit * 64)) 
    {
    // 如果 VBIAS 更大，手动减去 1 个 LSB 进行补偿
    AppPHCfg.DacData12Bit = AppPHCfg.DacData12Bit - 1; 
    }
    AppPHCfg.AdcClkFreq = 32000000.0;
    AppPHCfg.SysClkFreq = 32000000.0;
    AppPHCfg.DataRst = bFALSE;
    //VBIAS to LP PA; VZERO to 
    AppPHCfg.LpDacSW = LPDACSW_VBIAS2LPPA|LPDACSW_VBIAS2PIN|LPDACSW_VZERO2LPTIA|LPDACSW_VZERO2PIN;
    AppPHCfg.LpDacRef = LPDACREF_2P5;
    AppPHCfg.LpDacSrc = LPDACSRC_MMR;
    AppPHCfg.LpDacVbiasMux = LPDACVBIAS_12BIT;
    AppPHCfg.LpDacVzeroMux = LPDACVZERO_6BIT;
    AppPHCfg.PowerEn = bTRUE; 
    
    //LPTIA + LP PA
    AppPHCfg.LpAmpSel = LPAMP0;
    AppPHCfg.LpAmpPwrMod = LPAMPPWR_NORM;
    AppPHCfg.LpPaPwrEn = bTRUE;
    AppPHCfg.LpTiaPwrEn = bTRUE;
    // LPTIASW(12)|LPTIASW(13)
    AppPHCfg.LpTiaSW = LPTIASW(12)|LPTIASW(13)|LPTIASW(2)|LPTIASW(10)\
          |LPTIASW(5)|LPTIASW(9); /* Close these switches to make sure LP PA amplifier is closed loop */
    AppPHCfg.LpTiaRf = LPTIARF_SHORT;
    AppPHCfg.LpTiaRtia = LPTIARTIA_200R;
    AppPHCfg.LpTiaRload = LPTIARLOAD_100R;
    //HSloop
    AppPHCfg.DiodeClose = bFALSE;
    AppPHCfg.HstiaBias = HSTIABIAS_1P1;
    AppPHCfg.HstiaCtia = 16;
    AppPHCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
    AppPHCfg.HstiaDeRtia = HSTIADERTIA_OPEN;
    AppPHCfg.HstiaRtiaSel = HSTIARTIA_200;
    //switch Matrix
    AppPHCfg.DswitchSel = SWD_OPEN;
    AppPHCfg.PswitchSel = SWP_OPEN;
    AppPHCfg.NswitchSel = SWN_OPEN;
    AppPHCfg.TswitchSel = SWT_AIN0 | SWT_TRTIA;
    //ADC Filter
    return AD5940ERR_OK;

}

AD5940Err AppPHGetCfg(void *pCfg)
{
  if(pCfg)
  {
    *(AppPHCfg_Type**)pCfg = &AppPHCfg;
    return AD5940ERR_OK;
  }
  return AD5940ERR_PARA;
}
AD5940Err AppPHCtrl(int32_t BcmCtrl, void *pPara)
{
  switch(BcmCtrl)
  {
    case PHCTRL_START:
      {
        AD5940_SEQMmrTrig(SEQID_0);
        break;
      }
    case PHCTRL_STOPNOW:
      {
        AppPHCfg.StopRequired = bTRUE;
        break;
      }
    case PHCTRL_GETBIASVOLT:
      {
        *(float*)pPara = AppPHCfg.DacData12Bit;
      }
    case PHCTRL_GETZEROVOLT:
      {
        *(float*)pPara = AppPHCfg.DacData6Bit;
      }
    case PHCTRL_SHUTDOWN:
      {
      AppPHCtrl(PHCTRL_STOPNOW, 0);  /* 如果测量正在运行，则停止它。 */
      /* 关闭那些不受休眠操作自动控制的 LPloop 相关模块 */
      AFERefCfg_Type aferef_cfg;
      LPLoopCfg_Type lp_loop;
      memset(&aferef_cfg, 0, sizeof(aferef_cfg));
      AD5940_REFCfgS(&aferef_cfg);
      memset(&lp_loop, 0, sizeof(lp_loop));
      AD5940_LPLoopCfgS(&lp_loop);
      AD5940_EnterSleepS();  /* 进入休眠模式 */
      }
  }
  return AD5940ERR_OK;
}

AD5940Err AppPHSeqCfgGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  uint32_t const *pSeqCmd;
  uint32_t SeqLen;

  AFERefCfg_Type aferef_cfg;
  LPLoopCfg_Type lp_cfg;
  HSLoopCfg_Type hs_loop;
  ADCBaseCfg_Type adc_base;
  ADCFilterCfg_Type adc_filter;

  AD5940_SEQGenCtrl(bTRUE);
  AD5940_StructInit(&aferef_cfg,sizeof(aferef_cfg));
  //AD5940_AFECtrlS(AFECTRL_ALL, bFALSE); /* Init all to disable state*/
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

  AD5940_StructInit(&lp_cfg,sizeof(lp_cfg));
  lp_cfg.LpDacCfg.LpdacSel = AppPHCfg.LpdacSel;
  lp_cfg.LpDacCfg.DacData12Bit = AppPHCfg.DacData12Bit;
  lp_cfg.LpDacCfg.DacData6Bit = AppPHCfg.DacData6Bit;
  lp_cfg.LpDacCfg.DataRst = AppPHCfg.DataRst;
  lp_cfg.LpDacCfg.LpDacSW = AppPHCfg.LpDacSW;
  lp_cfg.LpDacCfg.LpDacRef = AppPHCfg.LpDacRef;
  lp_cfg.LpDacCfg.LpDacSrc = AppPHCfg.LpDacSrc;
  lp_cfg.LpDacCfg.LpDacVbiasMux = AppPHCfg.LpDacVbiasMux;
  lp_cfg.LpDacCfg.LpDacVzeroMux = AppPHCfg.LpDacVzeroMux;
  lp_cfg.LpDacCfg.PowerEn = AppPHCfg.PowerEn;
  lp_cfg.LpAmpCfg.LpAmpSel = AppPHCfg.LpAmpSel;
  lp_cfg.LpAmpCfg.LpAmpPwrMod = AppPHCfg.LpAmpPwrMod;
  lp_cfg.LpAmpCfg.LpTiaPwrEn = AppPHCfg.LpTiaPwrEn;
  lp_cfg.LpAmpCfg.LpPaPwrEn = AppPHCfg.LpPaPwrEn;
  lp_cfg.LpAmpCfg.LpTiaSW = AppPHCfg.LpTiaSW;
  lp_cfg.LpAmpCfg.LpTiaRf = AppPHCfg.LpTiaRf;
  lp_cfg.LpAmpCfg.LpTiaRtia = AppPHCfg.LpTiaRtia;
  lp_cfg.LpAmpCfg.LpTiaRload = AppPHCfg.LpTiaRload;
  AD5940_LPLoopCfgS(&lp_cfg);

  AD5940_StructInit(&hs_loop,sizeof(hs_loop));
  hs_loop.HsTiaCfg.DiodeClose = AppPHCfg.DiodeClose;
  hs_loop.HsTiaCfg.HstiaBias = AppPHCfg.HstiaBias;
  hs_loop.HsTiaCfg.HstiaCtia = AppPHCfg.HstiaCtia;
  hs_loop.HsTiaCfg.HstiaDeRload = AppPHCfg.HstiaDeRload;
  hs_loop.HsTiaCfg.HstiaDeRtia = AppPHCfg.HstiaDeRtia;
  hs_loop.HsTiaCfg.HstiaRtiaSel = AppPHCfg.HstiaRtiaSel;
  hs_loop.SWMatCfg.Dswitch = SWD_OPEN;
  hs_loop.SWMatCfg.Pswitch = SWP_PL|SWP_PL2;
  hs_loop.SWMatCfg.Nswitch = SWN_NL|SWN_NL2;
  hs_loop.SWMatCfg.Tswitch = SWT_AIN0 |SWT_TRTIA; //关键点1
  AD5940_HSLoopCfgS(&hs_loop);
  //V_out = 1.11V - I * R_TIA
  AD5940_StructInit(&adc_base,sizeof(adc_base));
  adc_base.ADCMuxN = ADCMUXN_VSET1P1;
  adc_base.ADCMuxP = ADCMUXP_HSTIA_P;
  adc_base.ADCPga  = ADCPGA_1;
  AD5940_ADCBaseCfgS(&adc_base);

  AD5940_StructInit(&adc_filter,sizeof(adc_filter));
  adc_filter.ADCAvgNum = ADCAVGNUM_16;
  adc_filter.ADCRate = ADCRATE_800KHZ;
  adc_filter.ADCSinc2Osr = ADCSINC2OSR_22;
  adc_filter.ADCSinc3Osr = ADCSINC3OSR_4;
  adc_filter.BpSinc3 = bFALSE;
  adc_filter.BpNotch = bTRUE;
  adc_filter.Sinc2NotchEnable = bFALSE;
  AD5940_ADCFilterCfgS(&adc_filter);
  



  AD5940_AFECtrlS(AFECTRL_HPREFPWR|AFECTRL_HSTIAPWR|AFECTRL_INAMPPWR|AFECTRL_EXTBUFPWR|\
    AFECTRL_DACREFPWR, bTRUE);


  AD5940_SEQGenInsert(SEQ_STOP());

  error = AD5940_SEQGenFetchSeq(&pSeqCmd,&SeqLen);
  AD5940_SEQGenCtrl(bFALSE);
  if (error == AD5940ERR_OK)
  {
    AppPHCfg.InitSeqInfo.SeqId = SEQID_1;
    AppPHCfg.InitSeqInfo.SeqRamAddr = AppPHCfg.SeqStartAddr;
    AppPHCfg.InitSeqInfo.pSeqCmd = pSeqCmd;
    AppPHCfg.InitSeqInfo.SeqLen = SeqLen;
    AD5940_SEQCmdWrite(AppPHCfg.InitSeqInfo.SeqRamAddr,pSeqCmd,SeqLen);
  }
  else
    return error;
  return AD5940ERR_OK;
}

AD5940Err AppPHSeqMeasureGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  uint32_t const *pSeqCmd;
  uint32_t SeqLen;

  uint32_t WaitClks;
  ClksCalInfo_Type clks_cal;

  clks_cal.DataType = DATATYPE_SINC3;
  clks_cal.DataCount = 1;
  clks_cal.ADCSinc3Osr = ADCSINC3OSR_4;
  clks_cal.ADCAvgNum = ADCAVGNUM_16;
  clks_cal.RatioSys2AdcClk = AppPHCfg.SysClkFreq/AppPHCfg.AdcClkFreq;;
  AD5940_ClksCalculate(&clks_cal,&WaitClks);
  WaitClks += 20;
  AD5940_SEQGenCtrl(bTRUE);
  AD5940_ADCMuxCfgS(ADCMUXP_HSTIA_P, ADCMUXN_VSET1P1);
  // 1. 开启 ADC 转换
  AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE);  /* 启用ADC电源 */
  AD5940_SEQGenInsert(SEQ_WAIT(16*80));
  AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE); 
  // 2. 等待转换完成
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks)); 
  // 3. 停止 ADC
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_ADCPWR, bFALSE);
    /* 序列结束。 */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* 停止序列发生器 */

  if(error == AD5940ERR_OK)
  {
    AppPHCfg.MeasureSeqInfo.SeqId = SEQID_0;
    AppPHCfg.MeasureSeqInfo.SeqRamAddr = AppPHCfg.InitSeqInfo.SeqRamAddr + AppPHCfg.InitSeqInfo.SeqLen ;
    AppPHCfg.MeasureSeqInfo.pSeqCmd = pSeqCmd;
    AppPHCfg.MeasureSeqInfo.SeqLen = SeqLen;
    /* 将命令写入 SRAM */
    AD5940_SEQCmdWrite(AppPHCfg.MeasureSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
  }
  else
    return error; /* 错误 */
  return AD5940ERR_OK;

}

AD5940Err AppPHInit(uint32_t *pBuffer, uint32_t BufferSize)
{
    AD5940Err error = AD5940ERR_OK;
    SEQCfg_Type seq_cfg;
    FIFOCfg_Type fifo_cfg;

    if(AD5940_WakeUp(10)>10)
      return AD5940ERR_WAKEUP;
    // 配置序列器
    seq_cfg.SeqMemSize = SEQMEMSIZE_2KB;
    seq_cfg.SeqBreakEn = bFALSE;
    seq_cfg.SeqIgnoreEn = bFALSE;
    seq_cfg.SeqCntCRCClr = bTRUE;
    seq_cfg.SeqEnable = bFALSE;
    seq_cfg.SeqWrTimer = 0;
    AD5940_SEQCfg(&seq_cfg);
    //重新初始化FIFO
    fifo_cfg.FIFOEn = bTRUE;
    fifo_cfg.FIFOMode = FIFOMODE_FIFO;
    fifo_cfg.FIFOSize = FIFOSIZE_4KB;
    fifo_cfg.FIFOSrc = FIFOSRC_SINC3; // ★ 改为 SINC3
    fifo_cfg.FIFOThresh = 1;          // 只要有1个数据就中断
    AD5940_FIFOCfg(&fifo_cfg);
    AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE);

    AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

    if((AppPHCfg.PHInited == bFALSE)||\
      (AppPHCfg.bParaChanged == bTRUE))
    {
        if(pBuffer == 0)return AD5940ERR_PARA;
        if(BufferSize == 0) return AD5940ERR_PARA;
        AD5940_SEQGenInit(pBuffer,BufferSize);
        //init seq
        error = AppPHSeqCfgGen();
        if(error != AD5940ERR_OK)return error;
        Serial.print("D\n");
        //measure seq
      /* 生成测量序列 */
        error = AppPHSeqMeasureGen();
        if(error != AD5940ERR_OK) return error;

        AppPHCfg.bParaChanged = bFALSE;
    }
    AppPHCfg.InitSeqInfo.WriteSRAM = bFALSE;
    AD5940_SEQInfoCfg(&AppPHCfg.InitSeqInfo);
    seq_cfg.SeqEnable = bTRUE;
    AD5940_SEQCfg(&seq_cfg);
    AD5940_SEQMmrTrig(AppPHCfg.InitSeqInfo.SeqId);
    while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_ENDSEQ) == bFALSE);
    AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
    AD5940_ClrMCUIntFlag(); 
    AppPHCfg.PHInited = bTRUE;
    return AD5940ERR_OK;
}

AD5940Err AppPHISR(void *pBuff, uint32_t *pCount)
{
    uint32_t BuffCount;
    uint32_t FIFOCnt;

    if(AppPHCfg.PHInited == bFALSE)
    return AD5940ERR_APPERROR;
    if(AD5940_WakeUp(10) > 10)  /* 通过读寄存器唤醒 AFE，最多读取 10 次 */
    return AD5940ERR_WAKEUP;  /* 唤醒失败 */

    AD5940_SleepKeyCtrlS(SLPKEY_LOCK);
    *pCount = 0;
    Serial.print("G");
    if(AD5940_INTCTestFlag(AFEINTC_0,AFEINTSRC_DATAFIFOTHRESH)==bTRUE)
    {
      FIFOCnt = AD5940_FIFOGetCnt();
      Serial.print("F");
      if(FIFOCnt > 0)
      {
        Serial.print("E");
        AD5940_FIFORd((uint32_t *)pBuff,FIFOCnt);
        *pCount = FIFOCnt;
      }
      AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
      AD5940_ClrMCUIntFlag();
    }
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
    return 0;
}
AD5940Err PHShowResult(uint32_t *pData, uint32_t DataCount)
{
  // 1. 定义常量
    const float VREF_ADC = 1.82f; // ADC 参考电压 (内部 1.82V)
    const float RTIA_VAL = 200.0f; // 你在 HSTIA 配置里选的是 10k (HSTIARTIA_10K)
    // 注意：如果你改了配置里的电阻，这里也要改！
    
    // 2. 遍历数据
    for(int i=0; i<DataCount; i++)
    {
        // 取出 16-bit 数据 (低 16 位有效，高位可能是通道 ID 或 0)
        uint16_t rawCode = pData[i] & 0xFFFF; 
        
        // 3. 转换为电压 (基于 ADC 配置: MuxN=1.11V, PGA=1)
        // AD5940 的 ADC 在 Sinc3 模式下，Code=0x8000 对应 0V 差分输入
        // 即：(Vin_P - Vin_N) = 0V
        // Code 转 Voltage 公式:
        float voltage_diff = ((float)rawCode - 32768.0f) / 32768.0f * VREF_ADC;
        
        // 4. 转换为电流
        // HSTIA 输出电压 Vout = Vbias - I * Rtia
        // ADC 测得的是 (Vout - Vbias) = - I * Rtia
        // 所以 I = - voltage_diff / Rtia
        float current_A = -voltage_diff / RTIA_VAL;
        float current_uA = current_A * 1e6f; // 转换为微安

        // 5. 打印
        Serial.printf("Index:%d, Code:0x%04X, VoltDiff: %.4f V, Current: %.4f uA\n", 
                      i, rawCode, voltage_diff, current_uA);
        
        // 6. (可选) 简单验证
        // 如果你接了 1k 电阻到 GND (假设 Vbias=1.1V):
        // 预期电流 I = 1.1V / 1k = 1.1mA = 1100uA
        // 如果接了 1k 电阻到 0.6V (CE0):
        // 预期电流 I = (1.1V - 0.6V) / 1k = 0.5mA = 500uA
    }
    return AD5940ERR_OK;
}


