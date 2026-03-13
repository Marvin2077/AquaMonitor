#include "command_dispatcher.h"
#include "app_globals.h"
#include <Arduino.h>

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
    default:
        // 暂时还没迁移的指令，让 main.cpp 的旧代码兜底处理
        break;
    }
}