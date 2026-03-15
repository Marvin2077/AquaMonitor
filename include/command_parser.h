#pragma once
#include <Arduino.h>

enum class CmdType {
    // 温度
    TEMP_READ,
    TEMP_CAL_P1,    // 25°C
    TEMP_CAL_P2,    // 35°C
    TEMP_CAL_P3,    // 50°C
    TEMP_SAVE,
    TEMP_RESET,
    TEMP_RESISTANCE,
    TEMP_GET_CALIB,
    TEMP_SET_CALIB,   // fParam 不够用，需要扩展 ParsedCommand 存 a/b/c
    
    // 电导率
    COND_INIT,
    COND_READ,
    COND_SET_STD,       // cond std <value>   → fParam = 标液值
    COND_CAL_KCELL,     // cond cal kcell
    COND_CAL_POINT,     // cond cal <value>   → fParam = 标液值
    COND_SAVE,
    COND_RESET,
    COND_SWEEP,
    COND_SET_FREQ,      // cond freq <value>  → fParam = 频率
    // pH
    PH_INIT,
    PH_READ,
    PH_CAL_OFFSET,
    PH_CAL_GAIN,      // fParam = 电阻值
    PH_SET_ISFET,     // iParam = 通道号 1-8
    //恢复出厂设置
    FACTORY_RESET,
    UNKNOWN
};

struct ParsedCommand {
    CmdType type   = CmdType::UNKNOWN;
    float   fParam = 0.0f;
    int     iParam = 0;
    bool    valid  = false;
    double dA = 0.0, dB = 1.0, dC = 0.0;
};

ParsedCommand parseCommand(const String& raw);