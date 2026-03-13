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
    UNKNOWN
};

struct ParsedCommand {
    CmdType type   = CmdType::UNKNOWN;
    float   fParam = 0.0f;
    int     iParam = 0;
    bool    valid  = false;
};

ParsedCommand parseCommand(const String& raw);