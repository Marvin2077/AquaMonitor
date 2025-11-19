#include "impedance_service.h"
#include "Arduino.h"

/* Default LPDAC resolution (2.5V internal reference)*/
#define DAC12BITVOLT_1LSB (2200.f/4095) //mV
#define DAC6BITVOLT_1LSB (DAC12BITVOLT_1LSB*64) //mv
AppIMPCfg_Type AppIMPCfg; 

AD5940Err AppIMPCfg_init(){

    AppIMPCfg.bParaChanged = bFALSE;
    AppIMPCfg.SeqStartAddr = 0;
    AppIMPCfg.MaxSeqLen = 0;
    AppIMPCfg.SeqStartADDrcal = 0;
    AppIMPCfg.MaxSeqLenCal = 0;
    //配置系统与测量频率
    AppIMPCfg.ImpODR = 20.0;
    AppIMPCfg.NumOfData = -1;
    AppIMPCfg.SysClkFreq = 16000000.0;
    AppIMPCfg.WuptClkFreq = 32000.0;
    AppIMPCfg.AdcClkFreq = 16000000.0;
    AppIMPCfg.RcalVal = 1000.0; //Ohm
    //配置开关矩阵
    AppIMPCfg.DswitchSel = SWD_CE0;
    AppIMPCfg.PswitchSel = SWP_CE0;
    AppIMPCfg.NswitchSel = SWN_AIN1;
    AppIMPCfg.TswitchSel = SWT_AIN1;
    
    AppIMPCfg.PwrMod = AFEPWR_HP;
    //HSTIA Configuration
    AppIMPCfg.HstiaRtiaSel = HSTIARTIA_5K;
    AppIMPCfg.ExcitBufGain = EXCITBUFGAIN_2;
    // HSDAC Configuration
    AppIMPCfg.HsDacGain = HSDACGAIN_1;
    AppIMPCfg.HsDacUpdateRate = 7;
    AppIMPCfg.DacVoltPP = 800.0;
    AppIMPCfg.BiasVolt = -0.0f;

    // DFT configuration
    AppIMPCfg.DftNum = DFTNUM_16384;
    AppIMPCfg.DftSrc = DFTSRC_SINC3;
    AppIMPCfg.HanwinEn = bTRUE;
    // ADC configuration
    AppIMPCfg.AdcPgaGain = ADCPGA_1;
    AppIMPCfg.ADCSinc3Osr = ADCSINC3OSR_2;
    AppIMPCfg.ADCSinc2Osr = ADCSINC2OSR_22,
    AppIMPCfg.ADCAvgNum = ADCAVGNUM_16;

    // sweep freq configuration
    AppIMPCfg.SinFreq = 100000.0;
    AppIMPCfg.SweepCfg.SweepEn = bTRUE;
    AppIMPCfg.SweepCfg.SweepStart = 1000;
    AppIMPCfg.SweepCfg.SweepStop = 100000.0;
    AppIMPCfg.SweepCfg.SweepPoints = 100;
    AppIMPCfg.SweepCfg.SweepLog = bFALSE;
    AppIMPCfg.SweepCfg.SweepIndex = 0;

    // FIFO configuration
    AppIMPCfg.FifoThresh = 4;
    AppIMPCfg.IMPInited = bFALSE;
    AppIMPCfg.StopRequired = bFALSE;
    return AD5940ERR_OK;

}

int32_t AppIMPGetCfg (void *pCfg)
{
    if(pCfg)
    {
        *(AppIMPCfg_Type**)pCfg = &AppIMPCfg;
        return AD5940ERR_OK;
    }
    return AD5940ERR_PARA;
}

int32_t AppIMPCtrl(uint32_t Command, void *pPara)
{
    switch(Command)
    {
        case IMPCTRL_START:
        {
           AD5940_SEQMmrTrig(SEQID_0);
           AppIMPCfg.FifoDataCount = 0;
        }
        case IMPCTRL_STOPNOW:
        {
            if(AD5940_WakeUp(10)>10)
            return AD5940ERR_WAKEUP;
            break;
        }
        case IMPCTRL_STOPSYNC:
        {
            AppIMPCfg.StopRequired = bTRUE;
            break;
        }
        case IMPCTRL_GETFREQ:
        {
            if(pPara == 0)
             return AD5940ERR_PARA;
            if(AppIMPCfg.SweepCfg.SweepEn == bTRUE)
             *(float*)pPara = AppIMPCfg.FreqofData;
             else
             *(float*)pPara = AppIMPCfg.SinFreq;
        }
        break;
        case IMPCTRL_SHUTDOWN:
        {
            AppIMPCtrl(IMPCTRL_STOPNOW,0);
            AFERefCfg_Type aferef_cfg;
            LPLoopCfg_Type lp_loop;
            memset(&aferef_cfg,0,sizeof(aferef_cfg));
            AD5940_REFCfgS(&aferef_cfg);
            memset(&lp_loop,0,sizeof(lp_loop));
            AD5940_LPLoopCfgS(&lp_loop);
            AD5940_EnterSleepS();
        }
        break;
        default:
        break;

    }
    return AD5940ERR_OK;
}

float AppIMPGetCurrFreq(void)
{
    if(AppIMPCfg.SweepCfg.SweepEn == bTRUE)
    {
        return AppIMPCfg.FreqofData;
    }
    else
        return AppIMPCfg.SinFreq;
}
//构建阻抗测量初始化序列
static AD5940Err AppIMPSeqCfgGen(void)
{
    AD5940Err error = AD5940ERR_OK;
    const uint32_t *pSeqCmd;
    uint32_t SeqLen;
    AFERefCfg_Type aferef_cfg;
    HSLoopCfg_Type HsLoopCfg;
    DSPCfg_Type dsp_cfg;
    float sin_freq;

    /*Strat sqquence generator here*/
    AD5940_SEQGenCtrl(bTRUE);

    AD5940_AFECtrlS(AFECTRL_ALL, bFALSE); /* Init all to disable state*/
    /* ADC/DAC/TIA reference and buffer */
    aferef_cfg.HpBandgapEn = bTRUE; //带隙基准就是芯片里一颗温度系数很小、很稳定的“参考电压源”
    aferef_cfg.Hp1V1BuffEn = bTRUE;
    aferef_cfg.Hp1V8BuffEn = bTRUE;
    aferef_cfg.Disc1V1Cap = bFALSE; /**< Discharge 1.1V capacitor. Short external 1.1V decouple capacitor to ground. Be careful when use this bit  */
    aferef_cfg.Disc1V8Cap = bFALSE;
    aferef_cfg.Hp1V8ThemBuff = bFALSE; /**< Thermal Buffer for internal 1.8V reference to AIN3 pin  */ 
    aferef_cfg.Hp1V8Ilimit = bFALSE; /**< Current limit for High power 1.8V reference buffer */
    aferef_cfg.Lp1V1BuffEn = bFALSE;
    aferef_cfg.Lp1V8BuffEn - bFALSE;
    /* LP reference control - turn off them to save power*/
    if(AppIMPCfg.BiasVolt != 0.0f)
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

    /** HSDAC Configure */
    HsLoopCfg.HsDacCfg.ExcitBufGain = AppIMPCfg.ExcitBufGain;
    HsLoopCfg.HsDacCfg.HsDacGain = AppIMPCfg.HsDacGain;
    HsLoopCfg.HsDacCfg.HsDacUpdateRate = AppIMPCfg.HsDacUpdateRate;
    /** HSTIA Configure */
    HsLoopCfg.HsTiaCfg.DiodeClose = bFALSE;
      if(AppIMPCfg.BiasVolt != 0.0f)
        HsLoopCfg.HsTiaCfg.HstiaBias = HSTIABIAS_VZERO0;
      else
        HsLoopCfg.HsTiaCfg.HstiaBias = HSTIABIAS_1P1;
    HsLoopCfg.HsTiaCfg.HstiaCtia = 31;
    HsLoopCfg.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
    HsLoopCfg.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_OPEN;
    HsLoopCfg.HsTiaCfg.HstiaRtiaSel = AppIMPCfg.HstiaRtiaSel;
    /*MUX configure*/
    HsLoopCfg.SWMatCfg.Dswitch = AppIMPCfg.DswitchSel;
    HsLoopCfg.SWMatCfg.Pswitch = AppIMPCfg.PswitchSel;
    HsLoopCfg.SWMatCfg.Nswitch = AppIMPCfg.NswitchSel;
    HsLoopCfg.SWMatCfg.Tswitch = SWT_TRTIA|AppIMPCfg.TswitchSel;
    /*Wave Generator configure*/
    HsLoopCfg.WgCfg.WgType = WGTYPE_SIN;
    HsLoopCfg.WgCfg.GainCalEn = bTRUE; /**< Enable Gain calibration */
    HsLoopCfg.WgCfg.OffsetCalEn = bTRUE; /**< Enable offset calibration */
    if(AppIMPCfg.SweepCfg.SweepEn == bTRUE)
    {
        AppIMPCfg.FreqofData = AppIMPCfg.SweepCfg.SweepStart;
        AppIMPCfg.SweepCurrFreq = AppIMPCfg.SweepCfg.SweepStart;
        AD5940_SweepNext(&AppIMPCfg.SweepCfg,&AppIMPCfg.SweepNextFreq);
        sin_freq = AppIMPCfg.SweepCurrFreq;
    }
    else
    {
        sin_freq = AppIMPCfg.SinFreq;
        AppIMPCfg.FreqofData = sin_freq;
    }
    HsLoopCfg.WgCfg.SinCfg.SinFreqWord = AD5940_WGFreqWordCal(sin_freq,AppIMPCfg.SysClkFreq); //把目标频率 sin_freq 换算成 DDS 频率控制字（WGFCW）
    HsLoopCfg.WgCfg.SinCfg.SinAmplitudeWord = (uint32_t)(AppIMPCfg.DacVoltPP/800.0f*2047 + 0.5f); //峰-峰值（mVpp）换成幅度寄存器（WGAMPLITUDE，11 位）的码字
    HsLoopCfg.WgCfg.SinCfg.SinOffsetWord = 0;
    HsLoopCfg.WgCfg.SinCfg.SinPhaseWord = 0;
    AD5940_HSLoopCfgS(&HsLoopCfg);
    if(AppIMPCfg.BiasVolt != 0.0f)
    {
        LPDACCfg_Type lpdac_cfg;

        lpdac_cfg.LpdacSel = LPDAC0;
        lpdac_cfg.LpDacVbiasMux = LPDACVBIAS_12BIT; /* Use Vbias to tuning BiasVolt. */
        lpdac_cfg.LpDacVzeroMux = LPDACVZERO_6BIT; /* Vbias-Vzero = BiasVolt */
        lpdac_cfg.DacData6Bit = 0x40>>1;
        if(AppIMPCfg.BiasVolt<-1100.0f) AppIMPCfg.BiasVolt = -1100.f +DAC12BITVOLT_1LSB;
        if(AppIMPCfg.BiasVolt>1100.0f) AppIMPCfg.BiasVolt = 1100.f + DAC12BITVOLT_1LSB;
        lpdac_cfg.DacData12Bit = (uint32_t)((AppIMPCfg.BiasVolt + 1100.0f)/DAC12BITVOLT_1LSB);
        lpdac_cfg.DataRst = bFALSE;
        lpdac_cfg.LpDacSW = LPDACSW_VBIAS2LPPA|LPDACSW_VBIAS2PIN|LPDACSW_VZERO2LPTIA|LPDACSW_VZERO2PIN|LPDACSW_VZERO2HSTIA;
        lpdac_cfg.LpDacRef = LPDACREF_2P5;
        lpdac_cfg.LpDacSrc = LPDACSRC_MMR; /* Use MMR data, we use LPDAC to generate bias voltage for LPTIA - the Vzero */
        lpdac_cfg.PowerEn = bTRUE;
        AD5940_LPDACCfgS(&lpdac_cfg);
    }
    dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_HSTIA_N;
    dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_HSTIA_P;
    dsp_cfg.ADCBaseCfg.ADCPga = AppIMPCfg.AdcPgaGain;

    memset(&dsp_cfg.ADCDigCompCfg, 0, sizeof(dsp_cfg.ADCDigCompCfg));

    dsp_cfg.ADCFilterCfg.ADCAvgNum = AppIMPCfg.ADCAvgNum;
    dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ; /* Tell filter block clock rate of ADC*/
    dsp_cfg.ADCFilterCfg.ADCSinc2Osr = AppIMPCfg.ADCSinc2Osr;
    dsp_cfg.ADCFilterCfg.ADCSinc3Osr = AppIMPCfg.ADCSinc3Osr;
    dsp_cfg.ADCFilterCfg.BpNotch = bTRUE;
    dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE;
    dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;
    dsp_cfg.DftCfg.DftNum = AppIMPCfg.DftNum;
    dsp_cfg.DftCfg.DftSrc = AppIMPCfg.DftSrc;
    dsp_cfg.DftCfg.HanWinEn = AppIMPCfg.HanwinEn;

    memset(&dsp_cfg.StatCfg, 0, sizeof(dsp_cfg.StatCfg));
    AD5940_DSPCfgS(&dsp_cfg);

    /* Enable all of them. They are automatically turned off during hibernate mode to save power */
    if(AppIMPCfg.BiasVolt == 0.0f)
    {
     AD5940_AFECtrlS(AFECTRL_HSTIAPWR|AFECTRL_INAMPPWR|AFECTRL_EXTBUFPWR|\
        AFECTRL_WG|AFECTRL_DACREFPWR|AFECTRL_HSDACPWR|\
        AFECTRL_SINC2NOTCH, bTRUE);
    }
    else
    {
     AD5940_AFECtrlS(AFECTRL_HSTIAPWR|AFECTRL_INAMPPWR|AFECTRL_EXTBUFPWR|\
        AFECTRL_WG|AFECTRL_DACREFPWR|AFECTRL_HSDACPWR|\
        AFECTRL_SINC2NOTCH|AFECTRL_DCBUFPWR, bTRUE);
    }
    /* Sequence end*/
    AD5940_SEQGenInsert(SEQ_STOP());
    /* Add one extra command to disable sequencer for initialization sequence because we only want it to run one time. */

    /* Stop Here */
    error = AD5940_SEQGenFetchSeq(&pSeqCmd,&SeqLen);
    AD5940_SEQGenCtrl(bFALSE);
    if(error == AD5940ERR_OK)
    {
        AppIMPCfg.InitSeqInfo.SeqId = SEQID_1;
        AppIMPCfg.InitSeqInfo.SeqRamAddr = AppIMPCfg.SeqStartAddr;
        AppIMPCfg.InitSeqInfo.pSeqCmd = pSeqCmd;
        AppIMPCfg.InitSeqInfo.SeqLen = SeqLen;
        /* Write command to SRAM */
        AD5940_SEQCmdWrite(AppIMPCfg.InitSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
    }
    else
      return error;
    return AD5940ERR_OK;
}
//构建阻抗测量序列
static AD5940Err AppIMPSeqMeasureGen(void)
{
    AD5940Err error = AD5940ERR_OK;
    const uint32_t *pSeqCmd;
    uint32_t SeqLen;

    uint32_t WaitClks;
    SWMatrixCfg_Type sw_cfg;
    ClksCalInfo_Type clks_cal;

    //给一个时钟计算函数准备参数，让它算出这次测量需要等多少个系统时钟周期
    clks_cal.DataType = DATATYPE_DFT;
    clks_cal.DftSrc = AppIMPCfg.DftSrc;
    clks_cal.DataCount = 1L<<(AppIMPCfg.DftNum+2);
    clks_cal.ADCSinc2Osr = AppIMPCfg.ADCSinc2Osr;
    clks_cal.ADCSinc3Osr = AppIMPCfg.ADCSinc3Osr;
    clks_cal.ADCAvgNum = AppIMPCfg.ADCAvgNum;
    clks_cal.RatioSys2AdcClk = AppIMPCfg.SysClkFreq/AppIMPCfg.AdcClkFreq;
    //计算出从开始采样到 DFT 结果真正准备好，需要多少个系统时钟周期，结果放到 WaitClks 里。
    AD5940_ClksCalculate(&clks_cal,&WaitClks);
    
    AD5940_SEQGenCtrl(bTRUE);
    AD5940_SEQGpioCtrlS(AGPIO_Pin2);// 拉高Pin2
    /*这个 GPIO 是 AD5940 Sequencer 在测量开始和结束时发出的时序标志信号，通常被用来在示波器上观察测量过程、
    验证 WaitClks 是否正确、或进行 MCU/外部设备同步。它对测量本身没有影响，只是调试辅助。*/
    AD5940_SEQGenInsert(SEQ_WAIT(16*250));
    //配置开关矩阵
    sw_cfg.Dswitch = SWD_RCAL0;
    sw_cfg.Pswitch = SWP_RCAL0;
    sw_cfg.Nswitch = SWN_RCAL1;
    sw_cfg.Tswitch = SWT_RCAL1|SWT_TRTIA;
    AD5940_SWMatrixCfgS(&sw_cfg);

    AD5940_AFECtrlS(AFECTRL_HSTIAPWR|AFECTRL_INAMPPWR|AFECTRL_EXTBUFPWR|\
                AFECTRL_WG|AFECTRL_DACREFPWR|AFECTRL_HSDACPWR|\
                AFECTRL_SINC2NOTCH, bTRUE);
    AD5940_AFECtrlS(AFECTRL_WG|AFECTRL_ADCPWR, bTRUE);  /* Enable Waveform generator */
    //
    AD5940_SEQGenInsert(SEQ_WAIT(16*10));
    AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT, bTRUE);  /* Start ADC convert and DFT */
    AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));
    //等待ADC和DFT完成处理
    AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_ADCCNV|AFECTRL_DFT|AFECTRL_WG, bFALSE);  /* Stop ADC convert and DFT */

    /* 将开关矩阵配置为测量阻抗的引脚 */
    sw_cfg.Dswitch = AppIMPCfg.DswitchSel;
    sw_cfg.Pswitch = AppIMPCfg.PswitchSel;
    sw_cfg.Nswitch = AppIMPCfg.NswitchSel;
    sw_cfg.Tswitch = SWT_TRTIA|AppIMPCfg.TswitchSel;
    AD5940_SWMatrixCfgS(&sw_cfg);
    AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_WG, bTRUE);  /* Enable Waveform generator */
    AD5940_SEQGenInsert(SEQ_WAIT(16*10));  //delay for signal settling DFT_WAIT
    AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT, bTRUE);  /* Start ADC convert and DFT */
    AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));  /* wait for first data ready */
    AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT|AFECTRL_WG|AFECTRL_ADCPWR, bFALSE);  /* Stop ADC convert and DFT */
    AD5940_AFECtrlS(AFECTRL_HSTIAPWR|AFECTRL_INAMPPWR|AFECTRL_EXTBUFPWR|\
                AFECTRL_WG|AFECTRL_DACREFPWR|AFECTRL_HSDACPWR|\
                AFECTRL_SINC2NOTCH, bFALSE);
    AD5940_SEQGpioCtrlS(0); /* 拉低Pin2 */

    AD5940_EnterSleepS();/* Goto hibernate */

    /* Sequence end. */
    error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
    AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */

    if(error == AD5940ERR_OK)
    {
        AppIMPCfg.MeasureSeqInfo.SeqId = SEQID_0;
        AppIMPCfg.MeasureSeqInfo.SeqRamAddr = AppIMPCfg.InitSeqInfo.SeqRamAddr + AppIMPCfg.InitSeqInfo.SeqLen ;
        AppIMPCfg.MeasureSeqInfo.pSeqCmd = pSeqCmd;
        AppIMPCfg.MeasureSeqInfo.SeqLen = SeqLen;
        /* Write command to SRAM */
        AD5940_SEQCmdWrite(AppIMPCfg.MeasureSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
    }
    else
        return error; /* Error */

    return AD5940ERR_OK;
}

int32_t AppIMPInit(uint32_t *pBuffer, uint32_t BufferSize)
{
    AD5940Err error = AD5940ERR_OK;
    SEQCfg_Type seq_cfg;
    FIFOCfg_Type fifo_cfg;

    if(AD5940_WakeUp(10)>10)
    return AD5940ERR_WAKEUP;
    // 配置序列器
    seq_cfg.SeqMemSize = SEQMEMSIZE_2KB;
    seq_cfg.SeqBreakEn = bFALSE;
    seq_cfg.SeqIgnoreEn = bTRUE;
    seq_cfg.SeqCntCRCClr = bTRUE;
    seq_cfg.SeqEnable = bFALSE;
    seq_cfg.SeqWrTimer = 0;
    AD5940_SEQCfg(&seq_cfg);

    //重新初始化FIFO
    AD5940_FIFOCtrlS(FIFOSRC_DFT, bFALSE);
    fifo_cfg.FIFOEn = bTRUE;
    fifo_cfg.FIFOMode = FIFOMODE_FIFO;
    fifo_cfg.FIFOSize = FIFOSIZE_4KB;
    fifo_cfg.FIFOSrc = FIFOSRC_DFT;
    fifo_cfg.FIFOThresh = AppIMPCfg.FifoThresh;
    AD5940_FIFOCfg(&fifo_cfg);
    AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

    //如果第一次运行或者参数变化 → 重建序列
    if((AppIMPCfg.IMPInited == bFALSE)||\
       (AppIMPCfg.bParaChanged == bTRUE))
       {
        if(pBuffer == 0) return AD5940ERR_PARA;
        if(BufferSize == 0) return AD5940ERR_PARA;
        AD5940_SEQGenInit(pBuffer, BufferSize);

        /*重新生成初始化序列*/
        error = AppIMPSeqCfgGen();
        if(error != AD5940ERR_OK) return error;
        /*生成测量序列*/
        error = AppIMPSeqMeasureGen();
        if(error != AD5940ERR_OK) return error;
        //清除“参数已改变”标记
        AppIMPCfg.bParaChanged = bFALSE;
       }
    
    /*运行初始化序列（只运行一次）*/
    AppIMPCfg.InitSeqInfo.WriteSRAM = bFALSE;
    AD5940_SEQInfoCfg(&AppIMPCfg.InitSeqInfo);
    seq_cfg.SeqEnable = bTRUE;
    AD5940_SEQCfg(&seq_cfg);
    AD5940_SEQMmrTrig(AppIMPCfg.InitSeqInfo.SeqId);
    while(AD5940_INTCTestFlag(AFEINTC_1,AFEINTSRC_ENDSEQ) == bFALSE);
    //AD5940 的全部初始化已经就绪
      /* Measurement sequence  */
    AppIMPCfg.MeasureSeqInfo.WriteSRAM = bFALSE;
    AD5940_SEQInfoCfg(&AppIMPCfg.MeasureSeqInfo);
    seq_cfg.SeqEnable = bTRUE;
    AD5940_SEQCfg(&seq_cfg);  /* Enable sequencer, and wait for trigger */
    AD5940_ClrMCUIntFlag();   /* Clear interrupt flag generated before */
    AD5940_AFEPwrBW(AppIMPCfg.PwrMod, AFEBW_250KHZ);

    AppIMPCfg.IMPInited = bTRUE;  /* IMP application has been initialized. */
    return AD5940ERR_OK;
}

int32_t AppIMPRegModify(int32_t * const pData, uint32_t *pDataCount)
{
    if(AppIMPCfg.SweepCfg.SweepEn) /* Need to set new frequency and set power mode */
    {
        // 调用 ad5940.c 的函数，将下一个频率(SweepNextFreq)写入寄存器
        AD5940_WGFreqCtrlS(AppIMPCfg.SweepNextFreq, AppIMPCfg.SysClkFreq);
    }
    return AD5940ERR_OK;
}
//数据处理
int32_t AppIMPDataProcess(int32_t * const pData, uint32_t *pDataCount)
{
  uint32_t DataCount = *pDataCount;
  uint32_t ImpResCount = DataCount/4;

  fImpPol_Type * const pOut = (fImpPol_Type*)pData;
  iImpCar_Type * pSrcData = (iImpCar_Type*)pData;

  *pDataCount = 0;

  DataCount = (DataCount/4)*4;/* We expect RCAL data together with Rz data. One DFT result has two data in FIFO, real part and imaginary part.  */

  /* Convert DFT result to int32_t type */
  for(uint32_t i=0; i<DataCount; i++)
  {
    pData[i] &= 0x3ffff; /* @todo option to check ECC */
    if(pData[i]&(1L<<17)) /* Bit17 is sign bit */
    {
      pData[i] |= 0xfffc0000; /* Data is 18bit in two's complement, bit17 is the sign bit */
    }
  }
  for(uint32_t i=0; i<ImpResCount; i++)
  {
    iImpCar_Type *pDftRcal, *pDftRz;

    pDftRcal = pSrcData++;
    pDftRz = pSrcData++;
    float RzMag,RzPhase;
    float RcalMag, RcalPhase;
    
    RcalMag = sqrt((float)pDftRcal->Real*pDftRcal->Real+(float)pDftRcal->Image*pDftRcal->Image);
    RcalPhase = atan2(-pDftRcal->Image,pDftRcal->Real);
    RzMag = sqrt((float)pDftRz->Real*pDftRz->Real+(float)pDftRz->Image*pDftRz->Image);
    RzPhase = atan2(-pDftRz->Image,pDftRz->Real);

    RzMag = RcalMag/RzMag*AppIMPCfg.RcalVal;
    RzPhase = RcalPhase - RzPhase;
    //printf("V:%d,%d,I:%d,%d ",pDftRcal->Real,pDftRcal->Image, pDftRz->Real, pDftRz->Image);
    
    pOut[i].Magnitude = RzMag;
    pOut[i].Phase = RzPhase;
  }
  *pDataCount = ImpResCount; 
  AppIMPCfg.FreqofData = AppIMPCfg.SweepCurrFreq;
  /* Calculate next frequency point */
  if(AppIMPCfg.SweepCfg.SweepEn == bTRUE)
  {
    AppIMPCfg.FreqofData = AppIMPCfg.SweepCurrFreq;
    AppIMPCfg.SweepCurrFreq = AppIMPCfg.SweepNextFreq;
    AD5940_SweepNext(&AppIMPCfg.SweepCfg, &AppIMPCfg.SweepNextFreq);
  }

  return 0;
}

int32_t AppIMPISR(void *pBuff, uint32_t *pCount)
{
  uint32_t BuffCount;
  uint32_t FifoCnt;
  BuffCount = *pCount;
  
  *pCount = 0;
  
  if(AD5940_WakeUp(10) > 10)  /* Wakeup AFE by read register, read 10 times at most */
    return AD5940ERR_WAKEUP;  /* Wakeup Failed */
  AD5940_SleepKeyCtrlS(SLPKEY_LOCK);  /* Prohibit AFE to enter sleep mode. */

  if(AD5940_INTCTestFlag(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH) == bTRUE)
  {
    /* Now there should be 4 data in FIFO */
    FifoCnt = (AD5940_FIFOGetCnt()/4)*4;
    
    if(FifoCnt > BuffCount)
    {
      ///@todo buffer is limited.
    }
    AD5940_FIFORd((uint32_t *)pBuff, FifoCnt);
    AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
    AppIMPRegModify((int32_t *)pBuff, &FifoCnt);   /* If there is need to do AFE re-configure, do it here when AFE is in active state */
    //AD5940_EnterSleepS(); /* Manually put AFE back to hibernate mode. This operation only takes effect when register value is ACTIVE previously */
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* Allow AFE to enter sleep mode. */
    /* Process data */ 
    AppIMPDataProcess((int32_t*)pBuff,&FifoCnt); 
    *pCount = FifoCnt;
    return 0;
  }
  
  return 0;
} 

int32_t ImpedanceShowResult(uint32_t *pData, uint32_t DataCount)
{
  float freq; // 用于存储当前频率值的变量

  // 将通用的 uint32_t 指针强制转换为 fImpPol_Type (浮点型极坐标阻抗数据) 指针
  // AD5940 库会将原始的 DFT 实部/虚部 结果处理成 极坐标形式(幅值/相位)
  fImpPol_Type *pImp = (fImpPol_Type*)pData;

  // 调用应用层控制函数，获取当前测量的频率 (IMPCTRL_GETFREQ)
  AppIMPCtrl(IMPCTRL_GETFREQ, &freq);

  printf("Freq:%.2f ", freq); // 打印当前频率

  /* 处理数据 */
  for(int i=0;i<DataCount;i++)
  {
    // 打印每个数据点的阻抗幅值 (Magnitude) 和相位 (Phase)
    // 相位数据原始单位是弧度 (Radian)，这里乘以 180/MATH_PI 转换为角度 (Degree)
    printf("RzMag: %f Ohm , RzPhase: %f \n",pImp[i].Magnitude,pImp[i].Phase*180/MATH_PI);
  }
  return 0;
}
