#pragma once
#include "temp_service.h"
#include "Conductivity_service.h"
#include "ph_service.h"

// === 系统状态机 ===
enum SystemState {
    STATE_IDLE,
    STATE_TEMP_MEASURE,
    STATE_TEMP_CAL_P1,
    STATE_TEMP_CAL_P2,
    STATE_TEMP_CAL_P3,
    STATE_TEMP_SAVE_CAL,
    STATE_TEMP_RESET_CAL,
    STATE_TEMP_RESISTANCE,
    STATE_COND_INIT,
    STATE_COND_MEASURE,
    STATE_COND_SWEEP,
    STATE_COND_CAL,
    STATE_PH_INIT,
    STATE_PH_CHANNEL,
    STATE_PH_MEASURE,
    STATE_PH_CAL_OFFSET,
    STATE_PH_CAL_GAIN
};

extern SystemState currentState;

// === ADS124S08 / 温度服务 ===
extern ADS124S08_Drv* ads124s08;
extern TempService*   g_tempSvc;
extern double         currentTemp;

// === 电导率 ===
extern bool    g_isCondMode;
extern bool    g_isSweepMode;
extern int     g_sweepCount;
extern int     g_sweepTotalPoints;
extern float   g_condStdValue;

// === pH ===
extern bool    g_ispHMode;
extern float   g_calResistorValue;
extern uint8_t g_isfetChannel;

// === AD5941 共用缓冲区 ===
#define APPBUFF_SIZE 512
extern uint32_t AppBuff[APPBUFF_SIZE];