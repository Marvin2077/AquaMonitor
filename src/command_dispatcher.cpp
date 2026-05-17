#include "command_dispatcher.h"
#include "app_globals.h"
#include <Arduino.h>
#include "mux_iface.h"
#include "storage_manager.h"
#include "protocol_writer.h"

void dispatchCommand(const ParsedCommand& cmd) {
    if (!cmd.valid) {
        Protocol::error("SYS", "UNKNOWN_CMD");
        return;
    }
    if (currentState != STATE_IDLE) {
        Protocol::error("SYS", "BUSY");
        return;
    }

    switch (cmd.type) {
    case CmdType::TEMP_READ:
        setSystemState(STATE_TEMP_MEASURE);
        break;
    case CmdType::TEMP_CAL_P1:
        setSystemState(STATE_TEMP_CAL_P1);
        break;
    case CmdType::TEMP_CAL_P2:
        setSystemState(STATE_TEMP_CAL_P2);
        break;
    case CmdType::TEMP_CAL_P3:
        setSystemState(STATE_TEMP_CAL_P3);
        break;
    case CmdType::TEMP_SAVE:
        setSystemState(STATE_TEMP_SAVE_CAL);
        break;
    case CmdType::TEMP_RESET:
        setSystemState(STATE_TEMP_RESET_CAL);
        break;
    case CmdType::TEMP_RESISTANCE:
        setSystemState(STATE_TEMP_RESISTANCE);
        break;
    case CmdType::TEMP_GET_CALIB:
    {
        TempService::CalibCoeff c = g_tempSvc->getCalib();
        Protocol::linef("$TEMP,CALIB,%.8f,%.8f,%.8f,%d*",
                      c.a, c.b, c.c, c.valid ? 1 : 0);
        break;
    }
    case CmdType::TEMP_SET_CALIB:
    {
        TempService::CalibCoeff c;
        c.a     = cmd.dA;
        c.b     = cmd.dB;
        c.c     = cmd.dC;
        c.valid = true;
        g_tempSvc->setCalib(c);
        saveTempParams((float)c.a, (float)c.b, (float)c.c, true);
        Protocol::line("$TEMP,CALIB,SET,OK*");
        break;
    }
    case CmdType::COND_INIT:
        setSystemState(STATE_COND_INIT);
        break;

    case CmdType::COND_READ:
        if (!g_isCondMode) { Protocol::error("COND", "NOT_IN_MODE"); return; }
        ChooseSenesingChannel(1);
        AppCondCtrl(CondCTRL_START, 0);
        setSystemState(STATE_COND_MEASURE);
        break;

    case CmdType::COND_SWEEP:
        if (!g_isCondMode) { Protocol::error("COND", "NOT_IN_MODE"); return; }
        if (g_isSweepMode) { Protocol::error("COND", "ALREADY_SWEEPING"); return; }
        ChooseSenesingChannel(1);
        AppCondCfg.SweepCfg.SweepEn = bTRUE;
        AppCondCfg.bParaChanged = bTRUE;
        if (AppCondInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK) {
            g_isSweepMode = true;
            g_sweepCount = 0;
            g_sweepTotalPoints = AppCondCfg.SweepCfg.SweepPoints;
            Protocol::linef("$COND,SWEEP_START,%d*", g_sweepTotalPoints);
            AppCondCtrl(CondCTRL_START, 0);
            setSystemState(STATE_COND_SWEEP);
        } else {
            Protocol::error("COND", "SWEEP_INIT_FAILED");
        }
        break;

    case CmdType::COND_SET_FREQ:
        if (!g_isCondMode) { Protocol::error("COND", "NOT_IN_MODE"); return; }
        AppCondCfg.SinFreq = cmd.fParam;
        AppCondCfg.bParaChanged = bTRUE;
        if (AppCondInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK) {
            Protocol::linef("$COND,FREQ_SET,%.2f*", cmd.fParam);
        } else {
            Protocol::error("COND", "FREQ_SET_FAILED");
        }
        break;
    case CmdType::COND_SET_VPP:
        if(!g_isCondMode) { Protocol::error("COND", "NOT_IN_MODE"); return; }
        AppCondCfg.DacVoltPP = cmd.fParam;
        AppCondCfg.bParaChanged = bTRUE;
        if (AppCondInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK) {
            Protocol::linef("$COND,VPP_SET,%.2f*", cmd.fParam);
        } else {
            Protocol::error("COND", "VPP_SET_FAILED");
        }
        break;

    case CmdType::PH_INIT:
        ChooseSenesingChannel(2);
        setSystemState(STATE_PH_INIT);
        break;

    case CmdType::PH_READ:
        if (!g_ispHMode) { Protocol::error("PH", "NOT_IN_MODE"); return; }
        ChooseSenesingChannel(2);
        AppPHCtrl(PHCTRL_START, 0);
        setSystemState(STATE_PH_MEASURE);
        break;

    case CmdType::PH_CAL_OFFSET:
        if (!g_ispHMode) { Protocol::error("PH", "NOT_IN_MODE"); return; }
        ChooseSenesingChannel(2);
        AppPHCfg.TswitchSel  = SWT_TRTIA;
        AppPHCfg.bParaChanged = bTRUE;
        if (AppPHInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK) {
            AppPHCtrl(PHCTRL_START, 0);
            setSystemState(STATE_PH_CAL_OFFSET);
            Protocol::line("$PH,CAL_OFFSET,START*");
        } else {
            Protocol::error("PH", "CAL_OFFSET_INIT_FAILED");
        }
        break;

    case CmdType::PH_CAL_GAIN:
        if (!g_ispHMode) { Protocol::error("PH", "NOT_IN_MODE"); return; }
        ChooseSenesingChannel(4);
        g_calResistorValue    = cmd.fParam;
        AppPHCfg.DswitchSel   = SWD_OPEN;
        AppPHCfg.PswitchSel   = SWP_PL | SWP_PL2;
        AppPHCfg.NswitchSel   = SWN_OPEN;
        AppPHCfg.TswitchSel   = SWT_AIN0 | SWT_TRTIA;
        AppPHCfg.bParaChanged  = bTRUE;
        if (AppPHInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK) {
            AppPHCtrl(PHCTRL_START, 0);
            setSystemState(STATE_PH_CAL_GAIN);
            Protocol::linef("$PH,CAL_GAIN,START,%.1f*", cmd.fParam);
        } else {
            Protocol::error("PH", "CAL_GAIN_INIT_FAILED");
        }
        break;

    case CmdType::PH_SET_ISFET:
        if (currentState != STATE_IDLE) { Protocol::error("SYS", "BUSY"); return; }
        g_isfetChannel = (uint8_t)cmd.iParam;
        setSystemState(STATE_PH_CHANNEL);
        break;

    case CmdType::FACTORY_RESET:
        resetCalibrationParams();
        AppCondCfg.K_Cell     = 1.0f;
        AppCondCfg.BiasVolt   = 0.0f;
        AppCondCfg.ADCAvgNum  = ADCAVGNUM_16;
        AppPHCfg.ZeroOffset_Code  = 32768;
        AppPHCfg.Rtia_Value_Ohm   = 160000.0f;
        {
            TempService::CalibCoeff clean;
            g_tempSvc->setCalib(clean);
        }
        Protocol::line("$SYS,FACTORY_RESET,OK*");
        break;
    default:

        break;
    }
}
