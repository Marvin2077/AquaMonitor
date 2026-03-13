#include "command_parser.h"

ParsedCommand parseCommand(const String& raw) {
    ParsedCommand cmd;

    if      (raw == "temp read")       { cmd.type = CmdType::TEMP_READ;       cmd.valid = true; }
    else if (raw == "temp cal 25")     { cmd.type = CmdType::TEMP_CAL_P1;     cmd.valid = true; }
    else if (raw == "temp cal 35")     { cmd.type = CmdType::TEMP_CAL_P2;     cmd.valid = true; }
    else if (raw == "temp cal 50")     { cmd.type = CmdType::TEMP_CAL_P3;     cmd.valid = true; }
    else if (raw == "temp save")       { cmd.type = CmdType::TEMP_SAVE;       cmd.valid = true; }
    else if (raw == "temp reset")      { cmd.type = CmdType::TEMP_RESET;      cmd.valid = true; }
    else if (raw == "temp resistance") { cmd.type = CmdType::TEMP_RESISTANCE; cmd.valid = true; }
    

    return cmd;
}