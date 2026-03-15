#include "state_processors.h"
#include "app_globals.h"
#include "temp_service.h"
#include "Conductivity_service.h"
#include "ph_service.h"
#include "storage_manager.h"
#include "mux_iface.h"
#include <Arduino.h>

extern uint32_t AppBuff[];   // 定义在 main.cpp

    void processTempMeasure(){
        g_tempSvc->measure(currentTemp);
        Serial.printf("$TEMP,MEAS,%.3f*\n", currentTemp);
        currentState = STATE_IDLE;
    }

    void processTempCalP1() {
        if (g_tempSvc->recordCalibPoint(0, 25.0)) {
            Serial.println("$TEMP,CAL_PT,1,25.0,OK*");
        } else {
            Serial.println("$ERR,TEMP,Point 1 Failed*");
        }
        currentState = STATE_IDLE;
    }

    void processTempCalP2() {
        if (g_tempSvc->recordCalibPoint(1, 35.0)) {
            Serial.println("$TEMP,CAL_PT,2,35.0,OK*");
        } else {
            Serial.println("Point 2 Failed!");
        }
        currentState = STATE_IDLE;
    }

    void processTempCalP3() {
        if (g_tempSvc->recordCalibPoint(2, 50.0)) {
            Serial.println("Point 3 Saved!");
        } else {
            Serial.println("Point 3 Failed!");
        }
        currentState = STATE_IDLE;
    }

    void processTempSaveCal() {
        if (g_tempSvc->finishCalibration()) {
            auto c = g_tempSvc->getCalib();
            saveTempParams(c.a, c.b, c.c, true);
            Serial.printf("Calibration DONE! a=%.6f, b=%.6f, c=%.6f\n", c.a, c.b, c.c);
        } else {
            Serial.println("Calibration Calculation Failed (Check points?)");
        }
        currentState = STATE_IDLE;
    }

    void processTempResetCal() {
        TempService::CalibCoeff clean;
        g_tempSvc->setCalib(clean);
        Serial.println("Calibration Cleared.");
        currentState = STATE_IDLE;
    }

    void processTempResistance() {
        double r_ohm = 0.0;
        if (g_tempSvc->readResistance(r_ohm)) {
            Serial.printf("$TEMP,RES,%.2f*\n", r_ohm);
        } else {
            Serial.println("[ERR] Resistance read failed");
        }
        currentState = STATE_IDLE;
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
            Serial.println("$COND,INIT,OK*");
        } else {
            Serial.println("$ERR,COND,Init failed*");
        }
        currentState = STATE_IDLE;
    }

    void processCondMeasure() {
        uint32_t tempCount = APPBUFF_SIZE;
        if (AppCondCfg.CondInited == bFALSE || g_isCondMode == false) {
            Serial.println("$ERR,COND,Not initialized*");
            currentState = STATE_IDLE;
            return;
        }
        if (AppCondISR(AppBuff, &tempCount) == 0) {
            if (tempCount > 0) {
                CondShowResult(AppBuff, tempCount, false, 0, 0);
                currentState = STATE_IDLE;
            }
        }
    }

    void processCondSweep() {
        uint32_t tempCount = APPBUFF_SIZE;
        if (AppCondCfg.CondInited == bFALSE || g_isCondMode == false) {
            g_isSweepMode = false;
            g_sweepCount  = 0;
            AppCondCfg.SweepCfg.SweepEn    = bFALSE;
            AppCondCfg.SweepCfg.SweepIndex = 0;
            Serial.println("$ERR,COND,Not initialized*");
            currentState = STATE_IDLE;
            return;
        }
        if (AppCondISR(AppBuff, &tempCount) == 0) {
            if (tempCount > 0) {
                CondShowResult(AppBuff, tempCount, true, g_sweepCount + 1, g_sweepTotalPoints);
                g_sweepCount++;
                if (g_sweepCount < g_sweepTotalPoints) {
                    AppCondCtrl(CondCTRL_START, 0);
                } else {
                    Serial.println("$COND,SWEEP_END*");
                    g_isSweepMode = false;
                    g_sweepCount  = 0;
                    AppCondCfg.SweepCfg.SweepIndex = 0;
                    AppCondCfg.SweepCfg.SweepEn    = bFALSE;
                    AppCondCfg.bParaChanged         = bTRUE;
                    AppCondInit(AppBuff, APPBUFF_SIZE);
                    currentState = STATE_IDLE;
                }
            }
        }
    }

    void processCondCal() {
        uint32_t tempCount = APPBUFF_SIZE;
        if (AppCondCfg.CondInited == bFALSE || g_isCondMode == false) {
            Serial.println("$ERR,COND,Not initialized*");
            currentState = STATE_IDLE;
            return;
        }
        if (AppCondISR(AppBuff, &tempCount) == 0) {
            if (tempCount > 0) {
                static uint8_t calSampleCount = 0;
                static float   calAccumG      = 0.0f;
                static float   calAccumT      = 0.0f;
                const  uint8_t CAL_SAMPLES    = 6;

                float measured_G = ComputeKCell(AppBuff, tempCount);
                if (measured_G <= 0.0f) {
                    Serial.println("$ERR,COND,CAL,Invalid G=0*");
                    calSampleCount = 0; calAccumG = 0.0f; calAccumT = 0.0f;
                    currentState = STATE_IDLE;
                    return;
                }

                g_tempSvc->measure(currentTemp);
                calAccumG += measured_G;
                calAccumT += currentTemp;
                calSampleCount++;

                Serial.printf("$COND,CAL,SAMPLE,%d/%d,G=%.4f,T=%.2f*\n",
                            calSampleCount, CAL_SAMPLES, measured_G, currentTemp);

                if (calSampleCount < CAL_SAMPLES) {
                    AppCondCtrl(CondCTRL_START, 0);
                } else {
                    float avg_G   = calAccumG / (float)CAL_SAMPLES;
                    float avg_T   = calAccumT / (float)CAL_SAMPLES;
                    float ec_true = ApecaStd1413_TrueEC(avg_T);
                    float new_K   = ec_true / avg_G;

                    AppCondCfg.K_Cell = new_K;
                    saveCondParams(new_K);
                    Serial.printf("$COND,CAL,DONE,%.4f,%.2f,%.2f,%.4f*\n",
                                avg_G, ec_true, avg_T, new_K);

                    calSampleCount = 0; calAccumG = 0.0f; calAccumT = 0.0f;
                    currentState = STATE_IDLE;
                }
            }
        }
    }

    // P1/P2/P3 三个 state 共用同一函数体，static 变量在函数内跨调用保持状态
    void processCondCalPoint() {
        uint32_t tempCount = APPBUFF_SIZE;

        static uint8_t sampleCount = 0;
        static float   accum       = 0.0f;
        const  uint8_t SAMPLES     = 10;

        if (AppCondCfg.CondInited == bFALSE || g_isCondMode == false) {
            Serial.println("$ERR,COND,Not initialized*");
            sampleCount = 0; accum = 0.0f;
            currentState = STATE_IDLE;
            return;
        }
        if (AppCondISR(AppBuff, &tempCount) == 0) {
            if (tempCount > 0) {
                float raw_G = ComputeKCell(AppBuff, tempCount);
                if (raw_G <= 0.0f) {
                    Serial.println("$ERR,COND,CAL_P,Invalid G=0, retrying*");
                    AppCondCtrl(CondCTRL_START, 0);
                    return;
                }
                float rawCond = raw_G * AppCondCfg.K_Cell;
                accum += rawCond;
                sampleCount++;
                if (sampleCount < SAMPLES) {
                    AppCondCtrl(CondCTRL_START, 0);
                } else {
                    float avgCond = accum / (float)SAMPLES;
                    g_condCalPoints[g_condCalSlot].cond_true = g_condCalStdValue;
                    g_condCalPoints[g_condCalSlot].cond_meas = avgCond;
                    g_condCalPoints[g_condCalSlot].recorded  = true;
                    Serial.printf("$COND,CAL_PT,%d,%.2f,%.4f*\n",
                                g_condCalSlot, g_condCalStdValue, avgCond);
                    g_condCalSlot++;
                    sampleCount = 0;
                    accum = 0.0f;
                    currentState = STATE_IDLE;
                }
            }
        }
    }

    void processCondSaveCal() {
        if (!g_condCalPoints[0].recorded ||
            !g_condCalPoints[1].recorded ||
            !g_condCalPoints[2].recorded) {
            int n = (int)g_condCalPoints[0].recorded +
                    (int)g_condCalPoints[1].recorded +
                    (int)g_condCalPoints[2].recorded;
            Serial.printf("$ERR,COND,Not all 3 points recorded (%d/3)*\n", n);
            currentState = STATE_IDLE;
            return;
        }
        if (CondFitThreePoint(g_condCalPoints[0], g_condCalPoints[1],
                            g_condCalPoints[2], g_condCalib)) {
            saveCondCalib(g_condCalib.a, g_condCalib.b, g_condCalib.c, true);
            Serial.printf("$COND,CAL_SAVE,%.6f,%.6f,%.6f*\n",
                        g_condCalib.a, g_condCalib.b, g_condCalib.c);
        } else {
            Serial.println("$ERR,COND,3-point fit failed*");
        }
        currentState = STATE_IDLE;
    }

    void processCondResetCal() {
        g_condCalib = CondCalibCoeff{};
        saveCondCalib(0.0f, 1.0f, 0.0f, false);
        for (int i = 0; i < 3; i++) {
            g_condCalPoints[i].recorded  = false;
            g_condCalPoints[i].cond_true = 0.0f;
            g_condCalPoints[i].cond_meas = 0.0f;
        }
        g_condCalSlot = 0;
        Serial.println("$COND,CAL_RESET,OK*");
        currentState = STATE_IDLE;
    }

    // =========================================================
    // pH
    // =========================================================

    void processPhInit() {
        ChooseSenesingChannel(3);
        ChooseISFETChannel(g_isfetChannel);
        g_isCondMode = false;
        g_ispHMode   = true;
        if (AppPHInit(AppBuff, APPBUFF_SIZE) == AD5940ERR_OK) {
            AppCondCfg.CondInited = bFALSE;
            Serial.println("$PH,INIT,OK*");
        } else {
            Serial.println("$ERR,PH,INIT_FAILED*");
        }
        currentState = STATE_IDLE;
    }

    void processPhMeasure() {
        uint32_t tempCount = APPBUFF_SIZE;
        if (AppPHCfg.PHInited == bFALSE || g_ispHMode == false) {
            Serial.println("$ERR,PH,NOT_INITIALIZED*");
            currentState = STATE_IDLE;
            return;
        }
        if (AppPHISR(AppBuff, &tempCount) == 0) {
            if (tempCount > 0) {
                PHShowResult(AppBuff, tempCount);
                Serial.printf("$PH,MEAS_DONE,%lu*\n", (unsigned long)tempCount);
                currentState = STATE_IDLE;
            }
        }
    }

    void processPhCalOffset() {
        uint32_t tempCount = APPBUFF_SIZE;
        if (AppPHCfg.PHInited == bFALSE || g_ispHMode == false) {
            Serial.println("$ERR,PH,NOT_INITIALIZED*");
            currentState = STATE_IDLE;
            return;
        }
        if (AppPHISR(AppBuff, &tempCount) == 0) {
            if (tempCount > 0) {
                uint16_t measured_offset = AppBuff[0] & 0xFFFF;
                AppPHCfg.ZeroOffset_Code = measured_offset;
                savePhParams(measured_offset, AppPHCfg.Rtia_Value_Ohm);
                Serial.printf("$PH,CAL_OFFSET,OK,%u*\n", (unsigned)measured_offset);
                AppPHCfg.TswitchSel   = SWT_AIN1 | SWT_TRTIA;
                AppPHCfg.bParaChanged  = bTRUE;
                AppPHInit(AppBuff, APPBUFF_SIZE);
                currentState = STATE_IDLE;
            }
        }
    }

    void processPhCalGain() {
        uint32_t tempCount = APPBUFF_SIZE;
        if (AppPHCfg.PHInited == bFALSE || g_ispHMode == false) {
            Serial.println("$ERR,PH,NOT_INITIALIZED*");
            currentState = STATE_IDLE;
            return;
        }
        if (AppPHISR(AppBuff, &tempCount) == 0) {
            if (tempCount > 0) {
                uint16_t rawCode    = AppBuff[0] & 0xFFFF;
                int32_t  diff_code  = (int32_t)rawCode - (int32_t)AppPHCfg.ZeroOffset_Code;
                float    voltage_diff = ((float)diff_code / 32768.0f) * 1.82f;
                float    abs_volt     = fabs(voltage_diff);
                if (abs_volt > 0.05f) {
                    float calculated_rtia = (abs_volt * g_calResistorValue) / 1.1f;
                    AppPHCfg.Rtia_Value_Ohm = calculated_rtia;
                    savePhParams(AppPHCfg.ZeroOffset_Code, calculated_rtia);
                    Serial.printf("$PH,CAL_GAIN,OK,%.2f,%.1f,%u,%.6f*\n",
                                calculated_rtia, g_calResistorValue,
                                (unsigned)rawCode, voltage_diff);
                } else {
                    Serial.printf("$ERR,PH,SIGNAL_LOW,%.6f*\n", abs_volt);
                }
                currentState = STATE_IDLE;
            }
        }
    }

    void processPhChannel() {
    ChooseISFETChannel(g_isfetChannel);
    Serial.printf("$PH,ISFET,OK,%d*\n", g_isfetChannel);
    currentState = STATE_IDLE;
    }