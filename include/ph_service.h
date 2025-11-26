#ifndef _PH_SERVICE_H_
#define _PH_SERVICE_H_

extern "C"{
#include "ad5940.h"
}
#include "stdio.h"
#include "string.h"

typedef struct 
{
    //General Setting
    BoolFlag bParaChanged;
    uint32_t SeqStartAddr;
    uint32_t MaxSeqLen;
    uint32_t SeqStartAddrCal;
    uint32_t MaxSeqLenCal;
    BoolFlag StopRequired; 
    float FreqofData;                             /* 最新采样数据的频率 */
    BoolFlag PHInited;                          /* 如果程序首次运行，则生成序列命令 */
    SEQInfo_Type InitSeqInfo;
    SEQInfo_Type MeasureSeqInfo;
    uint32_t FifoDataCount;        
    uint32_t FifoThresh;
    uint32_t SysClkFreq;
    uint32_t AdcClkFreq;
    //LPDAC Configuration
    uint32_t LpdacSel;        /**< Selectr from LPDAC0 or LPDAC1. LPDAC1 is only available on ADuCM355. */
    uint32_t LpDacSrc;        /**< LPDACSRC_MMR or LPDACSRC_WG. Note: HSDAC is always connects to WG. Disable HSDAC if there is need. */
    uint32_t LpDacVzeroMux;   /**< Select which DAC output connects to Vzero. 6Bit or 12Bit DAC */
    uint32_t LpDacVbiasMux;   /**< Select which DAC output connects to Vbias */
    uint32_t LpDacSW;         /**< LPDAC switch set. Only available from Si2 */
    uint32_t LpDacRef;        /**< Reference selection. Either internal 2.5V LPRef or AVDD. select from @ref LPDACREF_Const*/
    BoolFlag DataRst;         /**< Keep Reset register REG_AFE_LPDACDAT0DATA */
    BoolFlag PowerEn;         /**< Power up REG_AFE_LPDACDAT0 */
    uint16_t DacData12Bit;    /**< Data for 12bit DAC */
    uint16_t DacData6Bit;     /**< Data for 6bit DAC */

    //LPAmp
    uint32_t LpAmpSel;        /**< Select from LPAMP0 and LPAMP1. LPAMP1 is only available on ADuCM355. */
    uint32_t LpTiaRf;         /**< The one order RC filter resistor selection. Select from @ref LPTIARF_Const */
    uint32_t LpTiaRload;      /**< The Rload resistor right in front of LPTIA negative input terminal. Select from @ref LPTIARLOAD_Const*/
    uint32_t LpTiaRtia;       /**< LPTIA RTIA resistor selection. Set it to open(@ref LPTIARTIA_Const) when use external resistor. */
    uint32_t LpAmpPwrMod;     /**< Power mode for LP PA and LPTIA */
    uint32_t LpTiaSW;         /**< Set of switches, using macro LPTIASW() to close switch */
    BoolFlag LpPaPwrEn;       /**< Enable(bTRUE) or disable(bFALSE) power of PA(potential amplifier) */
    BoolFlag LpTiaPwrEn;      /**< Enable(bTRUE) or Disable(bFALSE) power of LPTIA amplifier */
    //HSloop Configuration
    uint32_t HstiaBias;         /**< When select Vzero as bias, the related switch(VZERO2HSTIA) at LPDAC should be closed */
    uint32_t HstiaRtiaSel;      /**< RTIA selection @ref HSTIARTIA_Const */
    uint32_t ExtRtia;           /**< Value of external RTIA*/
    uint32_t HstiaCtia;         /**< Set internal CTIA value from 1 to 32 pF */
    BoolFlag DiodeClose;        /**< Close the switch for internal back to back diode */
    uint32_t HstiaDeRtia;       /**< DE0 node RTIA selection @ref HSTIADERTIA_Const */
    uint32_t HstiaDeRload;      /**< DE0 node Rload selection @ref HSTIADERLOAD_Const */
	/* 开关配置 */
    uint32_t DswitchSel;
    uint32_t PswitchSel;
    uint32_t NswitchSel;
    uint32_t TswitchSel;
}AppPHCfg_Type;
extern AppPHCfg_Type AppPHCfg;

#define PHCTRL_START    0
#define PHCTRL_STOPNOW  1
#define PHCTRL_STOPSYNC 2
#define PHCTRL_GETBIASVOLT  3
#define PHCTRL_GETZEROVOLT  4
#define PHCTRL_SHUTDOWN 5

AD5940Err AppPHCfg_init(void);
AD5940Err AppPHGetCfg(void *pCfg);
AD5940Err AppPHInit(uint32_t *pBuffer, uint32_t BufferSize);
AD5940Err AppPHISR(void *pBuff, uint32_t *pCount);
AD5940Err AppPHCtrl(int32_t BcmCtrl, void *pPara);
AD5940Err PHShowResult(uint32_t *pData, uint32_t DataCount);
AD5940Err AppPHSeqCfgGen(void);
AD5940Err AppPHSeqMeasureGen(void);



#endif