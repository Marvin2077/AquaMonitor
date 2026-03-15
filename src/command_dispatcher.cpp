#include "command_dispatcher.h"
#include "app_globals.h"
#include <Arduino.h>
#include "mux_iface.h"
#include "storage_manager.h"

void dispatchCommand(const ParsedCommand& cmd) {
    if (!cmd.valid) {
        Serial.println("$ERR,UNKNOWN_CMD*");
        return;
    }
    // 忙状态保护（只对温度指令，后续扩展）
    if (currentState != STATE_IDLE) {
        Serial.println("$ERR,BUSY*");
        return;
    }

    switch (cmd.type) {
    case CmdType::TEMP_READ:
        Serial.println("[CMD] Measuring Temperature");
        currentState = STATE_TEMP_MEASURE;
        break;
    case CmdType::TEMP_CAL_P1:
        Serial.println("[CMD] Measuring Point 1 (25.0 C)...");
        currentState = STATE_TEMP_CAL_P1;
        break;
    case CmdType::TEMP_CAL_P2:
        Serial.println("[CMD] Measuring Point 2 (35.0 C)...");
        currentState = STATE_TEMP_CAL_P2;
        break;
    case CmdType::TEMP_CAL_P3:
        Serial.println("[CMD] Measuring Point 3 (50.0 C)...");
        currentState = STATE_TEMP_CAL_P3;
        break;
    case CmdType::TEMP_SAVE:
        Serial.println("[CMD] Computing calibration...");
        currentState = STATE_TEMP_SAVE_CAL;
        break;
    case CmdType::TEMP_RESET:
        Serial.println("[CMD] Resetting calibration...");
        currentState = STATE_TEMP_RESET_CAL;
        break;
    case CmdType::TEMP_RESISTANCE:
        Serial.println("[CMD] Reading raw resistance...");
        currentState = STATE_TEMP_RESISTANCE;
        break;
    case CmdType::COND_INIT:
        currentState = STATE_COND_INIT;
        break;

    case CmdType::COND_READ:
        if (!g_isCondMode) { Serial.println("$ERR,COND,Not in Conductivity Mode*"); return; }
        ChooseSenesingChannel(1);
        AppCondCtrl(CondCTRL_START, 0);
        currentState = STATE_COND_MEASURE;
        break;

    case CmdType::COND_SET_STD:
        g_condStdValue = cmd.fParam;
        Serial.printf("$COND,STD_SET,%.2f*\n", cmd.fParam);
        break; 

    case CmdType::COND_CAL_KCELL:
        if (!g_isCondMode) { Serial.println("$ERR,COND,Not in Conductivity Mode*"); return; }
        ChooseSenesingChannel(1);
        AppCondCtrl(CondCTRL_START, 0);
        currentState = STATE_COND_CAL;
        break;

    case CmdType::COND_CAL_POINT:
        if (!g_isCondMode) { Serial.println("$ERR,COND,Not in Conductivity Mode*"); return; }
        if (g_condCalSlot > 2) { Serial.println("$ERR,COND,All 3 points recorded*"); return; }
        g_condCalStdValue = cmd.fParam;
        ChooseSenesingChannel(1);
        AppCondCtrl(CondCTRL_START, 0);
        if      (g_condCalSlot == 0) currentState = STATE_COND_CAL_P1;
        else if (g_condCalSlot == 1) currentState = STATE_COND_CAL_P2;
        else                         currentState = STATE_COND_CAL_P3;
        Serial.printf("[CMD] Recording cond cal point %d, std=%.2f\n", g_condCalSlot + 1, cmd.fParam);
        break;

    case CmdType::COND_SAVE:
        currentState = STATE_COND_SAVE_CAL;
        break;

    case CmdType::COND_RESET:
        currentState = STATE_COND_RESET_CAL;
        break;

    case CmdType::COND_SWEEP:
        if (!g_isCondMode) { Serial.println("$ERR,COND,Not in Conductivity Mode*"); return; }
        if (g_isSweepMode) { Serial.println("$ERR,COND,Already sweeping*"); return; }
        ChooseSenesingChannel(1);
        AppCondCfg.SweepCfg.SweepEn = bTRUE;
        AppCondCfg.bParaChanged = bTRUE;
        if (AppCondInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK) {
            g_isSweepMode = true;
            g_sweepCount = 0;
            g_sweepTotalPoints = AppCondCfg.SweepCfg.SweepPoints;
            Serial.printf("$COND,SWEEP_START,%d*\n", g_sweepTotalPoints);
            AppCondCtrl(CondCTRL_START, 0);
            currentState = STATE_COND_SWEEP;
        } else {
            Serial.println("$ERR,COND,Sweep init failed*");
        }
        break;

    case CmdType::COND_SET_FREQ:
        if (!g_isCondMode) { Serial.println("$ERR,COND,Not in Conductivity Mode*"); return; }
        AppCondCfg.SinFreq = cmd.fParam;
        AppCondCfg.bParaChanged = bTRUE;
        if (AppCondInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK) {
            Serial.printf("$COND,FREQ_SET,%.2f*\n", cmd.fParam);
        } else {
            Serial.println("$ERR,COND,Frequency set failed*");
        }
        break;

    case CmdType::PH_INIT:
        ChooseSenesingChannel(3);
        currentState = STATE_PH_INIT;
        break;

    case CmdType::PH_READ:
        if (!g_ispHMode) { Serial.println("$ERR,PH,NOT_IN_PH_MODE*"); return; }
        ChooseSenesingChannel(3);
        AppPHCtrl(PHCTRL_START, 0);
        currentState = STATE_PH_MEASURE;
        break;

    case CmdType::PH_CAL_OFFSET:
        if (!g_ispHMode) { Serial.println("$ERR,PH,NOT_IN_PH_MODE*"); return; }
        ChooseSenesingChannel(3);
        AppPHCfg.TswitchSel  = SWT_TRTIA;
        AppPHCfg.bParaChanged = bTRUE;
        if (AppPHInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK) {
            AppPHCtrl(PHCTRL_START, 0);
            currentState = STATE_PH_CAL_OFFSET;
            Serial.println("$PH,CAL_OFFSET,START*");
        } else {
            Serial.println("$ERR,PH,CAL_OFFSET_INIT_FAILED*");
        }
        break;

    case CmdType::PH_CAL_GAIN:
        if (!g_ispHMode) { Serial.println("$ERR,PH,NOT_IN_PH_MODE*"); return; }
        ChooseSenesingChannel(3);
        g_calResistorValue    = cmd.fParam;
        AppPHCfg.DswitchSel   = SWD_OPEN;
        AppPHCfg.PswitchSel   = SWP_PL | SWP_PL2;
        AppPHCfg.NswitchSel   = SWN_OPEN;
        AppPHCfg.TswitchSel   = SWT_AIN1 | SWT_TRTIA;
        AppPHCfg.bParaChanged  = bTRUE;
        if (AppPHInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK) {
            AppPHCtrl(PHCTRL_START, 0);
            currentState = STATE_PH_CAL_GAIN;
            Serial.printf("$PH,CAL_GAIN,START,%.1f*\n", cmd.fParam);
        } else {
            Serial.println("$ERR,PH,CAL_GAIN_INIT_FAILED*");
        }
        break;

    case CmdType::PH_SET_ISFET:
        if (currentState != STATE_IDLE) { Serial.println("$ERR,BUSY*"); return; }
        g_isfetChannel = (uint8_t)cmd.iParam;
        ChooseISFETChannel(g_isfetChannel);
        Serial.printf("$PH,ISFET,OK,%d*\n", g_isfetChannel);
        break;

    case CmdType::FACTORY_RESET:
        resetCalibrationParams();
        AppCondCfg.K_Cell     = 1.0f;
        AppCondCfg.BiasVolt   = 0.0f;
        AppCondCfg.ADCAvgNum  = ADCAVGNUM_16;
        AppPHCfg.ZeroOffset_Code  = 32768;
        AppPHCfg.Rtia_Value_Ohm   = AppCondCfg.HstiaRtiaSel;
        {
            TempService::CalibCoeff clean;
            g_tempSvc->setCalib(clean);
        }
        Serial.println(">>> Factory Reset Complete. Please Restart or Init Sensors. <<<");
        break;
    default:

        break;
    }
}