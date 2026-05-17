#include "serial_command_reader.h"

SerialReadResult readSerialCommandLine(String& out) {
    static char buffer[128];
    static size_t len = 0;

    while (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            buffer[len] = '\0';
            out = buffer;
            out.trim();
            len = 0;
            return out.length() > 0 ? SerialReadResult::LINE_READY : SerialReadResult::NONE;
        }
        if (len >= sizeof(buffer) - 1) {
            len = 0;
            out = "";
            return SerialReadResult::OVERFLOW;
        }
        buffer[len++] = c;
    }

    return SerialReadResult::NONE;
}
