#include "state_processors.h"
#include "app_globals.h"
#include "temp_service.h"
#include "Conductivity_service.h"
#include "ph_service.h"
#include "storage_manager.h"
#include "mux_iface.h"
#include "protocol_writer.h"
#include <Arduino.h>

extern uint32_t AppBuff[];   // 定义在 main.cpp

namespace {
constexpr uint32_t COND_MEASURE_TIMEOUT_MS = 15000;
constexpr uint32_t COND_SWEEP_TIMEOUT_MS = 120000;
constexpr uint32_t PH_MEASURE_TIMEOUT_MS = 15000;
constexpr uint32_t PH_CAL_TIMEOUT_MS = 15000;

bool stateTimedOut(uint32_t timeoutMs) {
    return currentState != STATE_IDLE && (millis() - g_stateEnteredMs) > timeoutMs;
}
}

    void processTempMeasure(){
        if (g_tempSvc->measure(currentTemp)) {
            Protocol::linef("$TEMP,MEAS,%.3f*", currentTemp);
        } else {
            Protocol::error("TEMP", "MEASURE_FAILED");
        }
        setSystemState(STATE_IDLE);
    }

    void processTempCalP1() {
        if (g_tempSvc->recordCalibPoint(0, 25.0)) {
            Protocol::line("$TEMP,CAL_PT,1,25.0,OK*");
        } else {
            Protocol::error("TEMP", "CAL_PT1_FAILED");
        }
        setSystemState(STATE_IDLE);
    }

    void processTempCalP2() {
        if (g_tempSvc->recordCalibPoint(1, 35.0)) {
            Protocol::line("$TEMP,CAL_PT,2,35.0,OK*");
        } else {
            Protocol::error("TEMP", "CAL_PT2_FAILED");
        }
        setSystemState(STATE_IDLE);
    }

    void processTempCalP3() {
        if (g_tempSvc->recordCalibPoint(2, 50.0)) {
            Protocol::line("$TEMP,CAL_PT,3,50.0,OK*");
        } else {
            Protocol::error("TEMP", "CAL_PT3_FAILED");
        }
        setSystemState(STATE_IDLE);
    }

    void processTempSaveCal() {
        if (g_tempSvc->finishCalibration()) {
            auto c = g_tempSvc->getCalib();
            saveTempParams(c.a, c.b, c.c, true);
            Protocol::linef("$TEMP,CALIB,OK,%.6f,%.6f,%.6f*", c.a, c.b, c.c);
        } else {
            Protocol::error("TEMP", "CALIB_CALC_FAILED");
        }
        setSystemState(STATE_IDLE);
    }

    void processTempResetCal() {
        TempService::CalibCoeff clean;
        g_tempSvc->setCalib(clean);
        Protocol::line("$TEMP,CALIB,RESET,OK*");
        setSystemState(STATE_IDLE);
    }

    void processTempResistance() {
        double r_ohm = 0.0;
        if (g_tempSvc->readResistance(r_ohm)) {
            Protocol::linef("$TEMP,RES,%.2f*", r_ohm);
        } else {
            Protocol::error("TEMP", "RESISTANCE_READ_FAILED");
        }
        setSystemState(STATE_IDLE);
    }

    // =========================================================
    // 电导率
    // =========================================================

    void processCondInit() {
        g_isCondMode = true;
        g_ispHMode   = false;
        ChooseSenesingChannel(1);
        if (AppCondInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK) {
            AppPHCfg.PHInited = bFALSE;
            Protocol::line("$COND,INIT,OK*");
        } else {
            Protocol::error("COND", "INIT_FAILED");
        }
        setSystemState(STATE_IDLE);
    }

    void processCondMeasure() {
        uint32_t tempCount = APPBUFF_SIZE;
        if (AppCondCfg.CondInited == bFALSE || g_isCondMode == false) {
            Protocol::error("COND", "NOT_INITIALIZED");
            setSystemState(STATE_IDLE);
            return;
        }
        if (AppCondISR(AppBuff, &tempCount) == 0 && tempCount > 0) {
            CondShowResult(AppBuff, tempCount, false, 0, 0);
            setSystemState(STATE_IDLE);
            return;
        }
        if (stateTimedOut(COND_MEASURE_TIMEOUT_MS)) {
            Protocol::error("COND", "MEASURE_TIMEOUT");
            setSystemState(STATE_IDLE);
        }
    }

    void processCondSweep() {
        uint32_t tempCount = APPBUFF_SIZE;
        if (AppCondCfg.CondInited == bFALSE || g_isCondMode == false) {
            g_isSweepMode = false;
            g_sweepCount  = 0;
            AppCondCfg.SweepCfg.SweepEn    = bFALSE;
            AppCondCfg.SweepCfg.SweepIndex = 0;
            Protocol::error("COND", "NOT_INITIALIZED");
            setSystemState(STATE_IDLE);
            return;
        }
        if (AppCondISR(AppBuff, &tempCount) == 0 && tempCount > 0) {
            CondShowResult(AppBuff, tempCount, true, g_sweepCount + 1, g_sweepTotalPoints);
            g_sweepCount++;
            if (g_sweepCount < g_sweepTotalPoints) {
                AppCondCtrl(CondCTRL_START, 0);
            } else {
                Protocol::line("$COND,SWEEP_END*");
                g_isSweepMode = false;
                g_sweepCount  = 0;
                AppCondCfg.SweepCfg.SweepIndex = 0;
                AppCondCfg.SweepCfg.SweepEn    = bFALSE;
                AppCondCfg.bParaChanged         = bTRUE;
                AppCondInit(AppBuff, APPBUFF_SIZE);
                setSystemState(STATE_IDLE);
            }
            return;
        }
        if (stateTimedOut(COND_SWEEP_TIMEOUT_MS)) {
            Protocol::error("COND", "SWEEP_TIMEOUT");
            g_isSweepMode = false;
            g_sweepCount  = 0;
            AppCondCfg.SweepCfg.SweepEn    = bFALSE;
            AppCondCfg.SweepCfg.SweepIndex = 0;
            setSystemState(STATE_IDLE);
        }
    }


    // =========================================================
    // pH
    // =========================================================

    void processPhInit() {
        ChooseSenesingChannel(2);
        ChooseISFETChannel(g_isfetChannel);
        g_isCondMode = false;
        g_ispHMode   = true;
        if (AppPHInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK) {
            AppCondCfg.CondInited = bFALSE;
            Protocol::line("$PH,INIT,OK*");
        } else {
            Protocol::error("PH", "INIT_FAILED");
        }
        setSystemState(STATE_IDLE);
    }

    void processPhMeasure() {
        uint32_t tempCount = APPBUFF_SIZE;
        if (AppPHCfg.PHInited == bFALSE || g_ispHMode == false) {
            Protocol::error("PH", "NOT_INITIALIZED");
            setSystemState(STATE_IDLE);
            return;
        }
        if (AppPHISR(AppBuff, &tempCount) == 0 && tempCount > 0) {
            PHShowResult(AppBuff, tempCount);
            Protocol::linef("$PH,MEAS_DONE,%lu*", (unsigned long)tempCount);
            setSystemState(STATE_IDLE);
            return;
        }
        if (stateTimedOut(PH_MEASURE_TIMEOUT_MS)) {
            Protocol::error("PH", "MEASURE_TIMEOUT");
            setSystemState(STATE_IDLE);
        }
    }

    void processPhCalOffset() {
        uint32_t tempCount = APPBUFF_SIZE;
        if (AppPHCfg.PHInited == bFALSE || g_ispHMode == false) {
            Protocol::error("PH", "NOT_INITIALIZED");
            setSystemState(STATE_IDLE);
            return;
        }
        if (AppPHISR(AppBuff, &tempCount) == 0 && tempCount > 0) {
            uint16_t measured_offset = AppBuff[0] & 0xFFFF;
            AppPHCfg.ZeroOffset_Code = measured_offset;
            savePhParams(measured_offset, AppPHCfg.Rtia_Value_Ohm);
            Protocol::linef("$PH,CAL_OFFSET,OK,%u*", (unsigned)measured_offset);
            AppPHCfg.TswitchSel   = SWT_AIN0 | SWT_TRTIA;
            AppPHCfg.bParaChanged  = bTRUE;
            AppPHInit(AppBuff, APPBUFF_SIZE);
            setSystemState(STATE_IDLE);
            return;
        }
        if (stateTimedOut(PH_CAL_TIMEOUT_MS)) {
            Protocol::error("PH", "CAL_OFFSET_TIMEOUT");
            setSystemState(STATE_IDLE);
        }
    }

    void processPhCalGain() {
        uint32_t tempCount = APPBUFF_SIZE;

    if (AppPHCfg.PHInited == bFALSE || g_ispHMode == false) {
        Protocol::error("PH", "NOT_INITIALIZED");
        setSystemState(STATE_IDLE);
        return;
    }

    if (AppPHISR(AppBuff, &tempCount) == 0 && tempCount > 0) {
        uint32_t sum = 0;

        for (uint32_t i = 0; i < tempCount; i++) {
            sum += (AppBuff[i] & 0xFFFF);
        }

        uint16_t rawCode = (uint16_t)(sum / tempCount);

        int32_t diff_code = (int32_t)rawCode - (int32_t)AppPHCfg.ZeroOffset_Code;

        const float ADC_REF_V       = 1.82f;
        const float HSTIA_BIAS_V    = 1.1f;

        float voltage_diff = ((float)diff_code / 32768.0f) * ADC_REF_V;
        float abs_volt     = fabsf(voltage_diff);

        // 保护 1：信号过小，通常对应 Rcal 未接入、TMUX 通道错误、AIN0 未接入 HSTIA
        if (abs_volt < 0.05f) {
            Protocol::linef(
                "$ERR,PH,CAL_SIGNAL_LOW,%.6f,%u*",
                abs_volt,
                (unsigned)rawCode
            );
        }
        // 保护 2：信号过大，通常对应 Rcal 太小、HSTIA 输出接近饱和、offset 错误
        else if (abs_volt > 1.20f) {
            Protocol::linef(
                "$ERR,PH,CAL_SIGNAL_HIGH,%.6f,%u*",
                voltage_diff,
                (unsigned)rawCode
            );
        }
        else {
            float calculated_rtia =
                (abs_volt * g_calResistorValue) / HSTIA_BIAS_V;

            AppPHCfg.Rtia_Value_Ohm = calculated_rtia;
            savePhParams(AppPHCfg.ZeroOffset_Code, calculated_rtia);

            Protocol::linef(
                "$PH,CAL_GAIN,OK,%.2f,%.1f,%u,%.6f,%.6f*",
                calculated_rtia,
                g_calResistorValue,
                (unsigned)rawCode,
                voltage_diff,
                abs_volt
            );
        }

        // 校准后恢复 pH 测量路径：TMUX1109 channel 2 + AIN0
        ChooseSenesingChannel(2);

        AppPHCfg.DswitchSel   = SWD_OPEN;
        AppPHCfg.PswitchSel   = SWP_PL | SWP_PL2;
        AppPHCfg.NswitchSel   = SWN_OPEN;
        AppPHCfg.TswitchSel   = SWT_AIN0 | SWT_TRTIA;
        AppPHCfg.bParaChanged = bTRUE;
        AppPHInit(AppBuff, APPBUFF_SIZE);

        setSystemState(STATE_IDLE);
        return;
    }

    if (stateTimedOut(PH_CAL_TIMEOUT_MS)) {
        Protocol::error("PH", "CAL_GAIN_TIMEOUT");

        ChooseSenesingChannel(2);
        AppPHCfg.TswitchSel   = SWT_AIN0 | SWT_TRTIA;
        AppPHCfg.bParaChanged = bTRUE;
        AppPHInit(AppBuff, APPBUFF_SIZE);

        setSystemState(STATE_IDLE);
    }
    }

    void processPhChannel() {
    ChooseISFETChannel(g_isfetChannel);
    Protocol::linef("$PH,ISFET,OK,%d*", g_isfetChannel);
    setSystemState(STATE_IDLE);
    }
