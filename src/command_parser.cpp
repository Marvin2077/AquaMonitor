#include "command_parser.h"

static bool parseTripleDoubleParams(const String& params, ParsedCommand& cmd) {
    int comma1 = params.indexOf(',');
    int comma2 = params.indexOf(',', comma1 + 1);
    if (comma1 <= 0 || comma2 <= comma1) return false;

    cmd.dA = params.substring(0, comma1).toDouble();
    cmd.dB = params.substring(comma1 + 1, comma2).toDouble();
    cmd.dC = params.substring(comma2 + 1).toDouble();
    return true;
}

static ParsedCommand parseStructuredCommand(String raw) {
    ParsedCommand cmd;
    if (!raw.startsWith("$CMD,")) return cmd;

    if (raw.endsWith("*")) raw.remove(raw.length() - 1);

    int first = raw.indexOf(',');
    int second = raw.indexOf(',', first + 1);
    int third = raw.indexOf(',', second + 1);
    if (first < 0 || second < 0) return cmd;

    String module = raw.substring(first + 1, second);
    String action = third > 0 ? raw.substring(second + 1, third) : raw.substring(second + 1);
    String params = third > 0 ? raw.substring(third + 1) : "";

    module.toUpperCase();
    action.toUpperCase();

    if (module == "TEMP") {
        if (action == "READ") { cmd.type = CmdType::TEMP_READ; cmd.valid = true; }
        else if (action == "GET_CALIB") { cmd.type = CmdType::TEMP_GET_CALIB; cmd.valid = true; }
        else if (action == "SET_CALIB" && parseTripleDoubleParams(params, cmd)) {
            cmd.type = CmdType::TEMP_SET_CALIB;
            cmd.valid = true;
        }
    } else if (module == "COND") {
        if (action == "INIT") { cmd.type = CmdType::COND_INIT; cmd.valid = true; }
        else if (action == "READ") { cmd.type = CmdType::COND_READ; cmd.valid = true; }
        else if (action == "SWEEP") { cmd.type = CmdType::COND_SWEEP; cmd.valid = true; }
        else if (action == "SET_FREQ") {
            float v = params.toFloat();
            if (v >= 2000.0f && v <= 200000.0f) {
                cmd.type = CmdType::COND_SET_FREQ;
                cmd.fParam = v;
                cmd.valid = true;
            }
        } else if (action == "SET_VPP") {
            float vpp = params.toFloat();
            if (vpp >= 50.0f && vpp <= 800.0f) {
                cmd.type = CmdType::COND_SET_VPP;
                cmd.fParam = vpp;
                cmd.valid = true;
            }
        }
    } else if (module == "PH") {
        if (action == "INIT") { cmd.type = CmdType::PH_INIT; cmd.valid = true; }
        else if (action == "READ") { cmd.type = CmdType::PH_READ; cmd.valid = true; }
        else if (action == "CAL_OFFSET") { cmd.type = CmdType::PH_CAL_OFFSET; cmd.valid = true; }
        else if (action == "CAL_GAIN") {
            float v = params.toFloat();
            if (v > 0.0f) {
                cmd.type = CmdType::PH_CAL_GAIN;
                cmd.fParam = v;
                cmd.valid = true;
            }
        } else if (action == "SET_ISFET") {
            int ch = params.toInt();
            if (ch >= 1 && ch <= 8) {
                cmd.type = CmdType::PH_SET_ISFET;
                cmd.iParam = ch;
                cmd.valid = true;
            }
        }
    } else if (module == "SYS") {
        if (action == "FACTORY_RESET") { cmd.type = CmdType::FACTORY_RESET; cmd.valid = true; }
    }

    return cmd;
}

ParsedCommand parseCommand(const String& raw) {
    if (raw.startsWith("$CMD,")) return parseStructuredCommand(raw);

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
    else if (raw.startsWith("cond vpp ")){
        float vpp = raw.substring(9).toFloat();
        if (vpp>= 50.0f && vpp <= 800.0f) { cmd.type = CmdType::COND_SET_VPP; cmd.fParam = vpp; cmd.valid = true;}
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