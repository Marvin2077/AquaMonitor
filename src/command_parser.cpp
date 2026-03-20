#include "command_parser.h"

ParsedCommand parseCommand(const String& raw) {
    ParsedCommand cmd;
    // 温度
    if      (raw == "temp read")       { cmd.type = CmdType::TEMP_READ;       cmd.valid = true; }
    else if (raw == "temp cal 25")     { cmd.type = CmdType::TEMP_CAL_P1;     cmd.valid = true; }
    else if (raw == "temp cal 35")     { cmd.type = CmdType::TEMP_CAL_P2;     cmd.valid = true; }
    else if (raw == "temp cal 50")     { cmd.type = CmdType::TEMP_CAL_P3;     cmd.valid = true; }
    else if (raw == "temp save")       { cmd.type = CmdType::TEMP_SAVE;       cmd.valid = true; }
    else if (raw == "temp reset")      { cmd.type = CmdType::TEMP_RESET;      cmd.valid = true; }
    else if (raw == "temp resistance") { cmd.type = CmdType::TEMP_RESISTANCE; cmd.valid = true; }
    else if (raw == "GET_TEMP_CALIB")  { cmd.type = CmdType::TEMP_GET_CALIB; cmd.valid = true; }
    else if (raw.startsWith("SET_TEMP_CALIB ")) {                                                 // 新增
        // 格式: SET_TEMP_CALIB <a> <b> <c>
        String params = raw.substring(15); // 跳过 "SET_TEMP_CALIB "
        params.trim();
        int sp1 = params.indexOf(' ');
        int sp2 = params.indexOf(' ', sp1 + 1);
        if (sp1 > 0 && sp2 > sp1) {
            cmd.dA = params.substring(0, sp1).toDouble();
            cmd.dB = params.substring(sp1 + 1, sp2).toDouble();
            cmd.dC = params.substring(sp2 + 1).toDouble();
            cmd.type  = CmdType::TEMP_SET_CALIB;
            cmd.valid = true;
        }
    }

    // 电导率
    else if (raw == "cond init")      { cmd.type = CmdType::COND_INIT;  cmd.valid = true; }
    else if (raw == "cond read")      { cmd.type = CmdType::COND_READ;  cmd.valid = true; }
    else if (raw == "cond sweep")     { cmd.type = CmdType::COND_SWEEP; cmd.valid = true; }
    else if (raw.startsWith("cond freq ")) {
        float v = raw.substring(10).toFloat();
        if (v >= 2000.0f && v <= 200000.0f) { cmd.type = CmdType::COND_SET_FREQ; cmd.fParam = v; cmd.valid = true; }
    }
    //pH值
    else if (raw == "ph init")        { cmd.type = CmdType::PH_INIT;       cmd.valid = true; }
    else if (raw == "ph read")        { cmd.type = CmdType::PH_READ;       cmd.valid = true; }
    else if (raw == "ph cal offset")  { cmd.type = CmdType::PH_CAL_OFFSET; cmd.valid = true; }

    else if (raw.startsWith("ph cal gain")) {
        String sub = raw.substring(11);
        sub.trim();
        float v = sub.toFloat();
        if (v > 0.0f) { cmd.type = CmdType::PH_CAL_GAIN; cmd.fParam = v; cmd.valid = true; }
    }
    else if (raw.startsWith("ph isfet ")) {
        int ch = raw.substring(9).toInt();
        if (ch >= 1 && ch <= 8) { cmd.type = CmdType::PH_SET_ISFET; cmd.iParam = ch; cmd.valid = true; }
    }
    
    else if (raw == "factory reset") { cmd.type = CmdType::FACTORY_RESET; cmd.valid = true; }
    return cmd;
}